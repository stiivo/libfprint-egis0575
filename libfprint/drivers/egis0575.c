/*
 * Egis Technology Inc. (aka. LighTuning) 0575 driver for libfprint
 * Copyright (C) 2021 Animesh Sahu <animeshsahu19@yahoo.com>
 * Copyright (C) 2022 Nils Schötteler <schoetni1@gmail.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

/*
 * As the driver is configured, it needs a long swipe of the whole fingertip to produce a good image.
 * It can be configured by changeing the values of EGIS0575_CONSECUTIVE_CAPTURES and EGIS0575_CAPTURE_DELAY.
 */

#define FP_COMPONENT "egis0575"

#include <math.h>

#include "egis0575.h"
#include "drivers_api.h"

/*
 * ==================== Basic definitions ====================
 */

/* Struct to share data across lifecycle */
struct _FpDeviceEgis0575
{
  FpImageDevice   parent;

  gboolean        running;
  gboolean        stop;

  GSList         *strips;
  gsize           strips_len;

  unsigned char * calibration_sequence;
  int             calibration_retries;

  const Packet   *pkt_array;
  int             pkt_array_len;
  int             current_index;
  int             img_without_finger_in_row;
};

// Helper State machine
// Sends all packages in pkt_array to the Sensor
enum packet_ssm_states {
  PACKET_SSM_REQ, // Sends Package at current_index
  PACKET_SSM_RESP, // Recieves Response and increments current_index
  PACKET_SSM_LOOP, // if current_index == pkt_array_len: to next state; else: loops to PACKET_SSM_REQ
  PACKET_SSM_DONE // final State, SSM finishes
};

// Setup State Mashine
// Runs at the start, if calibration pkg is in storage, this loads it to the calibration_sequence. if not it gets it from sensor and stores it.
enum setup_ssm_states {
  SSM_SETUP_START,
  SSM_PRE_CALIBRATION_BYTES_PHASE_1,
  SSM_PRE_CALIBRATION_BYTES_PHASE_2_REQ,
  SSM_PRE_CALIBRATION_BYTES_PHASE_2_RESP,
  SSM_PRE_CALIBRATION_BYTES_PHASE_3,
  SSM_PRE_CALIBRATION_BYTES_PHASE_4_REQ,
  SSM_PRE_CALIBRATION_BYTES_PHASE_4_RESP,
  SSM_PRE_CALIBRATION_BYTES_PHASE_5,
  SSM_GET_CALIBRATION_BYTES_REQ,
  SSM_GET_CALIBRATION_BYTES_RESP,
  SSM_CHECK_CALIBRATION_BYTES,
  SSM_SETUP_DONE
};


// State machine for Initilisation
// State names base on wild guess on what everything does
enum inti_ssm_states {
  SSM_PRE_RESET,          // pre reset pacages get send
  SSM_RESET_REQ,          // waits untli all is reset
  SSM_RESET_RESP,
  SSM_POST_RESET,         // sends post reset packages
  SSM_CALIBRATION_REQ_1,  // sends the two calibration packages
  SSM_CALIBRATION_REQ_2,
  SSM_CALIBRATION_RESP,
  SSM_POST_CALIBRATION,   // sends after calibration, gets first picture
  SSM_POST_REPEAT,
  SSM_INIT_DONE
};

// State machine for getting img from Sensor
enum img_ssm_states {
  IMG_SSM_FINGER_REQ,       // Asks Sensor if it thinks a finger is prensent (has some kind of sensing that, not always right)
  IMG_SSM_FINGER_RESP,      // If Sensor thinks it is present then goes to next state, otherwise jump delayed to previous state
  IMG_SSM_PRE_FIRST_IMAGE,  // Sends EGIS0575_PRE_FIRST_IMAGE_PACKETS with packet_ssm
  IMG_SSM_FIRST_IMAGE_REQ,  // Sends Request for first image (different request for first image of sequence)
  IMG_SSM_FIRST_IMAGE_RESP, // Recieves first image
  IMG_SSM_PRE_REPEAT_IMAGE, // Sends EGIS0575_REPEAT_PACKETS with packet_ssm
  IMG_SSM_REPEAT_IMAGE_REQ, // Sends Request for the image for all following images from sequence
  IMG_SSM_REPEAT_IMAGE_RESP, // Recieves image + image post processing. jumps to IMG_SSM_PRE_REPEAT_IMAGE with EGIS0575_CAPTURE_DELAY, until EGIS0575_CONSECUTIVE_CAPTURES framecount is reached.
  IMG_SSM_POST_REPEAT,      // Sends EGIS0575_POST_REPEAT_PACKETS with packet_ssm
  IMG_SSM_PROCESS_DATA,     // If there is data it is processed and released. if there is no data then it jumps to IMG_SSM_FINGER_REQ
  IMG_SSM_FREE_DATA,
  IMG_SSM_DONE              // final State, SSM finishes
};

G_DECLARE_FINAL_TYPE (FpDeviceEgis0575, fpi_device_egis0575, FPI, DEVICE_EGIS0575, FpImageDevice);
G_DEFINE_TYPE (FpDeviceEgis0575, fpi_device_egis0575, FP_TYPE_IMAGE_DEVICE);

static unsigned char
egis_get_pixel (struct fpi_frame_asmbl_ctx *ctx, struct fpi_frame *frame, unsigned int x, unsigned int y)
{
  return frame->data[x + y * ctx->frame_width];
}

static struct fpi_frame_asmbl_ctx assembling_ctx = {
  .frame_width = EGIS0575_IMGWIDTH,
  .frame_height = EGIS0575_RFMGHEIGHT,
  .image_width = (EGIS0575_IMGWIDTH / 3) * 4,   /* PIXMAN expects width/stride to be multiple of 4 */
  .get_pixel = egis_get_pixel,
};

/*
 * ==================== Data processing ====================
 */

/* finger_present checks in software if there is a finger in the frame or not.
 * The sensor has also some kind of dedection hardware, but it is not always reliabel.
 * This function sums the square of the differece between neighbouring pixels. With no Finger on the Sensor this value is between 25 and 125.
 * With a Finger on the Sensor it is from 125 to 400.
 * If the value is over EGIS0575_MIN_SD the function returns true, otherwise false. */
static gboolean
finger_present (FpiUsbTransfer *transfer)
{
  unsigned char *buffer = transfer->buffer;
  int length = transfer->actual_length;

  double variance = 0;


  for (size_t i = 1; i < length; i++)
    variance += (buffer[i] - buffer[i - 1]) * (buffer[i] - buffer[i - 1]);
  variance /= length;

  fp_dbg ("%f", variance);


  return variance > EGIS0575_MIN_SD && variance < EGIS0575_MAX_SD;
}

/* Only the central rows of each strip end up in the image: the rows at the
 * border of the sensor have a lot less contrast. */
#define EGIS0575_BLIT_FIRST_ROW 10
#define EGIS0575_BLIT_ROWS 30

static unsigned char
egis_get_blit_pixel (struct fpi_frame_asmbl_ctx *ctx, struct fpi_frame *frame, unsigned int x, unsigned int y)
{
  return frame->data[x + (y + EGIS0575_BLIT_FIRST_ROW) * ctx->frame_width];
}

static struct fpi_frame_asmbl_ctx blit_ctx = {
  .frame_width = EGIS0575_IMGWIDTH,
  .frame_height = EGIS0575_BLIT_ROWS,
  .image_width = (EGIS0575_IMGWIDTH / 3) * 4,
  .get_pixel = egis_get_blit_pixel,
};

/*
 * Movement estimation
 *
 * The generic fpi_do_movement_estimation() does not work well for this sensor:
 * every frame has a strong brightness gradient (bright center, dark borders)
 * and some fixed pattern lines, which dominate the pixel differences and make
 * the estimated offsets jump between the limits of the search range.
 * So frames are high-pass filtered first and compared using the normalized
 * cross correlation, which only looks at the ridge structure.
 */
#define EGIS0575_HP_RADIUS 5
#define EGIS0575_MIN_OVERLAP 10
#define EGIS0575_MAX_DX 8
#define EGIS0575_MIN_CORRELATION 0.3

static float *
highpass_frame (struct fpi_frame *frame)
{
  const int w = assembling_ctx.frame_width, h = assembling_ctx.frame_height;
  const int r = EGIS0575_HP_RADIUS;
  g_autofree double *integral = g_new0 (double, (w + 1) * (h + 1));
  float *out = g_new (float, w * h);

  for (int y = 0; y < h; y++)
    for (int x = 0; x < w; x++)
      integral[(y + 1) * (w + 1) + x + 1] = frame->data[y * w + x] +
                                            integral[y * (w + 1) + x + 1] +
                                            integral[(y + 1) * (w + 1) + x] -
                                            integral[y * (w + 1) + x];

  for (int y = 0; y < h; y++)
    for (int x = 0; x < w; x++)
      {
        int x0 = MAX (x - r, 0), x1 = MIN (x + r + 1, w);
        int y0 = MAX (y - r, 0), y1 = MIN (y + r + 1, h);
        double sum = integral[y1 * (w + 1) + x1] - integral[y0 * (w + 1) + x1] -
                     integral[y1 * (w + 1) + x0] + integral[y0 * (w + 1) + x0];

        out[y * w + x] = frame->data[y * w + x] - sum / ((x1 - x0) * (y1 - y0));
      }

  return out;
}

/* Correlation of cur[y][x] with prev[y + dy][x + dx] over the overlapping area */
static double
correlation (const float *prev, const float *cur, int dx, int dy)
{
  const int w = assembling_ctx.frame_width, h = assembling_ctx.frame_height;
  double sa = 0, sb = 0, saa = 0, sbb = 0, sab = 0, n, cov, va, vb;

  for (int y = MAX (0, -dy); y < MIN (h, h - dy); y++)
    for (int x = MAX (0, -dx); x < MIN (w, w - dx); x++)
      {
        double a = prev[(y + dy) * w + x + dx], b = cur[y * w + x];
        sa += a;
        sb += b;
        saa += a * a;
        sbb += b * b;
        sab += a * b;
      }

  n = (double) (h - ABS (dy)) * (w - ABS (dx));
  cov = sab - sa * sb / n;
  va = saa - sa * sa / n;
  vb = sbb - sb * sb / n;
  if (va <= 0 || vb <= 0)
    return 0;
  return cov / sqrt (va * vb);
}

/* direction: 0 = search both directions, otherwise only dy with that sign (or 0) */
static double
find_offset (const float *prev, const float *cur, int direction, int *dx_out, int *dy_out)
{
  const int max_dy = assembling_ctx.frame_height - EGIS0575_MIN_OVERLAP;
  double best = -2;

  for (int dy = direction > 0 ? 0 : -max_dy; dy <= (direction < 0 ? 0 : max_dy); dy++)
    for (int dx = -EGIS0575_MAX_DX; dx <= EGIS0575_MAX_DX; dx++)
      {
        double c = correlation (prev, cur, dx, dy);
        if (c > best)
          {
            best = c;
            *dx_out = dx;
            *dy_out = dy;
          }
      }

  return best;
}

/* Replaces the frame with its high-pass filtered version, scaled to a fixed
 * contrast, so the strips have no brightness steps in the assembled image */
static void
normalize_frame (struct fpi_frame *frame, const float *filtered)
{
  const int size = assembling_ctx.frame_width * assembling_ctx.frame_height;
  double rms = 0;

  for (int i = 0; i < size; i++)
    rms += filtered[i] * filtered[i];
  rms = sqrt (rms / size) + 1e-6;

  for (int i = 0; i < size; i++)
    frame->data[i] = CLAMP (128 + 40 * filtered[i] / rms, 0, 255);
}

/* Estimates the movement between the strips and drops strips that can not be
 * matched reliably. Returns the new list of strips. */
static GSList *
movement_estimation (GSList *strips)
{
  g_autoptr(GPtrArray) filtered = g_ptr_array_new_with_free_func (g_free);
  double weighted_dy = 0;
  int direction, dx, dy;
  GSList *l, *kept;
  float *prev;

  for (l = strips; l != NULL; l = l->next)
    g_ptr_array_add (filtered, highpass_frame (l->data));

  /* First pass: find the swipe direction from the confident neighbour pairs */
  for (guint i = 1; i < filtered->len; i++)
    {
      double c = find_offset (filtered->pdata[i - 1], filtered->pdata[i], 0, &dx, &dy);
      if (c >= EGIS0575_MIN_CORRELATION)
        weighted_dy += c * dy;
    }
  direction = weighted_dy < 0 ? -1 : 1;

  /* Second pass: offsets in swipe direction only, compared to the last kept strip */
  kept = g_slist_prepend (NULL, strips->data);
  prev = filtered->pdata[0];
  normalize_frame (strips->data, prev);
  for (l = strips->next, dy = 1; l != NULL; l = l->next, dy++)
    {
      struct fpi_frame *frame = l->data;
      float *cur = filtered->pdata[dy];
      int fdx = 0, fdy = 0;
      double c = find_offset (prev, cur, direction, &fdx, &fdy);

      fp_dbg ("strip offset dy=%d dx=%d correlation %.2f", fdy, fdx, c);
      if (c < EGIS0575_MIN_CORRELATION)
        {
          g_free (frame);
          continue;
        }

      frame->delta_x = fdx;
      frame->delta_y = fdy;
      normalize_frame (frame, cur);
      kept = g_slist_prepend (kept, frame);
      prev = cur;
    }

  g_slist_free (strips);
  return g_slist_reverse (kept);
}

/* function borrowed from elan.c driver "elan_process_frame_linear" to make the stitching easier */
static void
process_frame_linear (unsigned char *raw_frame, GSList ** frames)
{
  unsigned int frame_size = assembling_ctx.frame_width * assembling_ctx.frame_height;
  struct fpi_frame *frame = g_malloc (frame_size + sizeof (struct fpi_frame));

  unsigned short min = 0xffff, max = 0;

  for (int i = 0; i < frame_size; i++)
    {
      if (raw_frame[i] < min)
        min = raw_frame[i];
      if (raw_frame[i] > max)
        max = raw_frame[i];
    }

  g_assert (max != min);

  unsigned short px;

  for (int i = 0; i < frame_size; i++)
    {
      px = raw_frame[i];
      px = (px - min) * 0xff / (max - min);
      frame->data[i] = (unsigned char) px;
    }

  *frames = g_slist_prepend (*frames, frame);
}

/*Processes the frames to a Image*/
static void
process_imgs (FpiSsm *ssm, FpDevice *dev)
{
  FpImageDevice *img_self = FP_IMAGE_DEVICE (dev);
  FpDeviceEgis0575 *self = FPI_DEVICE_EGIS0575 (dev);

  FpiImageDeviceState state;

  g_object_get (dev, "fpi-image-device-state", &state, NULL);
  if (state == FPI_IMAGE_DEVICE_STATE_CAPTURE)
    {
      if (!self->stop)
        {
          g_autoptr(FpImage) img = NULL;
          self->strips = g_slist_reverse (self->strips);

          /* Debugging aid: dump the raw strips of every swipe to $EGIS0575_DUMP_DIR */
          const char *dump_dir = g_getenv ("EGIS0575_DUMP_DIR");
          if (dump_dir)
            {
              gsize frame_size = assembling_ctx.frame_width * assembling_ctx.frame_height;
              g_autofree char *path = g_strdup_printf ("%s/swipe-%" G_GINT64_FORMAT ".raw", dump_dir, g_get_real_time ());
              g_autoptr(GByteArray) data = g_byte_array_new ();

              for (GSList *l = self->strips; l != NULL; l = l->next)
                g_byte_array_append (data, ((struct fpi_frame *) l->data)->data, frame_size);
              if (g_file_set_contents (path, (char *) data->data, data->len, NULL))
                fp_dbg ("Dumped %u strips to %s", (guint) self->strips_len, path);
            }

          self->strips = movement_estimation (self->strips);
          self->strips_len = g_slist_length (self->strips);

          img = fpi_assemble_frames (&blit_ctx, self->strips);
          img->flags |= (FPI_IMAGE_COLORS_INVERTED | FPI_IMAGE_PARTIAL);

          FpImage *resizedImage = fpi_image_resize (img, EGIS0575_RESIZE, EGIS0575_RESIZE);

          fpi_image_device_image_captured (img_self, resizedImage);
        }

      fpi_image_device_report_finger_status (img_self, FALSE);
    }
}

/*
 * ==================== IO ====================
 */
static void
resp_setup (FpiUsbTransfer *transfer, FpDevice *dev, gpointer user_data, GError *error)
{
  // error handling
  if(error)
    {
      fp_dbg ("Error occurred in setup");
      fpi_ssm_mark_failed (transfer->ssm, error);

      return;
    }

  // response handling
  switch(fpi_ssm_get_cur_state (transfer->ssm))
    {
    case SSM_PRE_CALIBRATION_BYTES_PHASE_2_RESP:
      if(transfer->buffer[5] != 0x05)  // Wait till value is 0x05
        fpi_ssm_jump_to_state (transfer->ssm, SSM_PRE_CALIBRATION_BYTES_PHASE_2_REQ);
      else
        fpi_ssm_next_state (transfer->ssm);
      break;

    case SSM_PRE_CALIBRATION_BYTES_PHASE_4_RESP:
      if(transfer->buffer[5] != 0x00) // Wait till value is 0x00
        fpi_ssm_jump_to_state (transfer->ssm, SSM_PRE_CALIBRATION_BYTES_PHASE_4_REQ);
      else
        fpi_ssm_next_state (transfer->ssm);
      break;

    default:
      fp_dbg ("This should never occure, this code should be called if ssm not in state 2 or 4 resp");
      fpi_ssm_mark_failed (transfer->ssm, g_error_new_literal (G_IO_ERROR, G_IO_ERROR_CANCELLED, "Fatal Error in Setup, wrong ssm state"));
      break;
    }




}
static void
resp_init (FpiUsbTransfer *transfer, FpDevice *dev, gpointer user_data, GError *error)
{
  // error handling
  if(error)
    {
      fp_dbg ("Error occurred in init");
      fpi_ssm_mark_failed (transfer->ssm, error);

      return;
    }

  // response handling
  if(transfer->buffer[5] == 0x00)  // Wait as long as value is 0
    fpi_ssm_jump_to_state (transfer->ssm, SSM_RESET_REQ);
  else  // if value is not 0 anymore, continue
    fpi_ssm_next_state (transfer->ssm);

}

static void
resp_finger_present (FpiUsbTransfer *transfer, FpDevice *dev, gpointer user_data, GError *error)
{
  // error handling
  if(error)
    {
      fpi_ssm_mark_failed (transfer->ssm, error);
      return;
    }

  if(transfer->buffer[5] <= 0x03) // if value is less then 0x01 then there is no finger on the sensor -> jump back with a little delay
    fpi_ssm_jump_to_state_delayed (transfer->ssm, IMG_SSM_FINGER_REQ, 10);
  else // value is bigger then 0x01 -> finger present, capture frames
    fpi_ssm_next_state (transfer->ssm);
}

static void
resp_image (FpiUsbTransfer *transfer, FpDevice *dev, gpointer user_data, GError *error)
{
  FpImageDevice *img_self = FP_IMAGE_DEVICE (dev);
  FpDeviceEgis0575 *self = FPI_DEVICE_EGIS0575 (dev);

  // error handling
  if(error)
    {
      fpi_ssm_mark_failed (transfer->ssm, error);
      return;
    }

  // check if finger is presen in the frame

  if(!finger_present (transfer))
    {
      self->img_without_finger_in_row += 1;
      // Many images in a row without a finger -> Finger probably not on Sensor -> Clear images of finger taken previously
      if(self->img_without_finger_in_row > EGIS0575_MAX_CAPTURES_WITHOUT_FINGER_IN_ROW)
        {
          fp_dbg ("To many images without Finger.");
          // Short swipe -> enough frames anyway, process them instead of throwing them away
          if(self->strips_len >= EGIS0575_MIN_CAPTURES)
            {
              fpi_ssm_jump_to_state (transfer->ssm, IMG_SSM_POST_REPEAT);
              return;
            }
          if(self->img_without_finger_in_row == EGIS0575_MAX_CAPTURES_WITHOUT_FINGER_IN_ROW + 1) // dont spam that
            fpi_image_device_report_finger_status (img_self, FALSE);
          g_slist_free_full (self->strips, g_free);
          self->strips_len = 0;
          self->strips = NULL;
          fpi_ssm_jump_to_state (transfer->ssm, IMG_SSM_POST_REPEAT); // end imges in row and wait till finger is on sensor again
          return;
        }

      fpi_ssm_jump_to_state (transfer->ssm, IMG_SSM_PRE_REPEAT_IMAGE);
      return;
    }
  else
    {
      if(self->strips_len == 0) // dont spam that
        fpi_image_device_report_finger_status (img_self, TRUE);
      // save frame to Buffer
      // process_frame_linear taken from clan.c driver, to make the job of stiching img together easier for libfprint
      process_frame_linear (transfer->buffer, &self->strips);
      self->strips_len += 1;
      self->img_without_finger_in_row = 0;
    }

  if(fpi_ssm_get_cur_state (transfer->ssm) == IMG_SSM_REPEAT_IMAGE_RESP && self->strips_len < EGIS0575_CONSECUTIVE_CAPTURES)
    fpi_ssm_jump_to_state_delayed (transfer->ssm, IMG_SSM_PRE_REPEAT_IMAGE, EGIS0575_CAPTURE_DELAY); // jump with a little delay so finger can move
  else
    fpi_ssm_next_state (transfer->ssm); // if EGIS0575_CONSECUTIVE_CAPTURES frames are captured go to processing
}

/*
 * ==================== SSM loopback ====================
 */


static void
packet_ssm_run_state (FpiSsm *ssm, FpDevice *dev)
{
  fpi_ssm_silence_debug (ssm);

  FpDeviceEgis0575 *self = FPI_DEVICE_EGIS0575 (dev);
  FpiUsbTransfer *transfer;

  switch(fpi_ssm_get_cur_state (ssm))
    {
    case PACKET_SSM_REQ: // sends current package
      Packet pkt = self->pkt_array[self->current_index];
      transfer = fpi_usb_transfer_new (dev);

      fpi_usb_transfer_fill_bulk_full (transfer, EGIS0575_EPOUT, pkt.sequence, pkt.length, NULL);

      transfer->ssm = ssm;
      transfer->short_is_error = TRUE;

      fpi_usb_transfer_submit (transfer, EGIS0575_TIMEOUT, NULL, fpi_ssm_usb_transfer_cb, NULL);
      break;

    case PACKET_SSM_RESP: // recevies response
      transfer = fpi_usb_transfer_new (dev);

      fpi_usb_transfer_fill_bulk (transfer, EGIS0575_EPIN, self->pkt_array[self->current_index].response_length);

      transfer->ssm = ssm;

      fpi_usb_transfer_submit (transfer, EGIS0575_TIMEOUT, NULL, fpi_ssm_usb_transfer_cb, NULL);
      self->current_index++;
      break;

    case PACKET_SSM_LOOP: // looks if it is complete
      if(self->current_index == self->pkt_array_len)
        fpi_ssm_mark_completed (ssm);
      else
        fpi_ssm_jump_to_state (ssm, PACKET_SSM_REQ);
      break;
    }
}

static void
loop_complete (FpiSsm *ssm, FpDevice *dev, GError *error)
{
  FpImageDevice *img_dev = FP_IMAGE_DEVICE (dev);
  FpDeviceEgis0575 *self = FPI_DEVICE_EGIS0575 (dev);

  self->running = FALSE;

  /* A pending stop has to be completed however the loop ended, otherwise
   * the device would hang in the deactivating state forever */
  if (self->stop)
    {
      g_clear_error (&error);
      fpi_image_device_deactivate_complete (img_dev, NULL);
    }
  else if (error)
    {
      fpi_image_device_session_error (img_dev, error);
    }
}

static void
setup_ssm_run_state (FpiSsm *ssm, FpDevice *dev)
{
  FpDeviceEgis0575 *self = FPI_DEVICE_EGIS0575 (dev);
  FpiUsbTransfer *transfer;
  FpiSsm *child_ssm;

  switch(fpi_ssm_get_cur_state (ssm))
    {
    case SSM_SETUP_START:

      self->calibration_sequence = g_malloc (sizeof (unsigned char) * EGIS0575_IMGSIZE);

      fpi_ssm_next_state (ssm);

      /*
         if (egis0575_load_calibration(self))
         fpi_ssm_mark_completed (ssm);
         else
         fpi_ssm_next_state (ssm);*/

      /* this is pseudo code for the case we get harddisk storage of the bytes
         if(packet already stored){
         read data and store it into self->calibration_sequence;
         jump to done;
         }
       */

      break;

    case SSM_PRE_CALIBRATION_BYTES_PHASE_1:

      self->pkt_array = EGIS0575_GET_CALIBRATION_BYTES_PACKETS_PHASE_1;
      self->pkt_array_len = G_N_ELEMENTS (EGIS0575_GET_CALIBRATION_BYTES_PACKETS_PHASE_1);
      self->current_index = 0;
      child_ssm = fpi_ssm_new (FP_DEVICE (dev), packet_ssm_run_state, PACKET_SSM_DONE);
      fpi_ssm_start_subsm (ssm, child_ssm);
      break;

    case SSM_PRE_CALIBRATION_BYTES_PHASE_2_REQ:
      transfer = fpi_usb_transfer_new (dev);

      fpi_usb_transfer_fill_bulk_full (transfer, EGIS0575_EPOUT, EGIS0575_GET_CALIBRATION_BYTES_PACKETS_PHASE_2.sequence, EGIS0575_GET_CALIBRATION_BYTES_PACKETS_PHASE_2.length, NULL);

      transfer->ssm = ssm;
      transfer->short_is_error = TRUE;

      fpi_usb_transfer_submit (transfer, EGIS0575_TIMEOUT, NULL, fpi_ssm_usb_transfer_cb, NULL);
      break;

    case SSM_PRE_CALIBRATION_BYTES_PHASE_2_RESP:
      transfer = fpi_usb_transfer_new (dev);

      fpi_usb_transfer_fill_bulk (transfer, EGIS0575_EPIN, EGIS0575_GET_CALIBRATION_BYTES_PACKETS_PHASE_2.response_length);

      transfer->ssm = ssm;
      transfer->short_is_error = TRUE;

      fpi_usb_transfer_submit (transfer, EGIS0575_TIMEOUT, NULL, resp_setup, NULL);
      break;

    case SSM_PRE_CALIBRATION_BYTES_PHASE_3:

      self->pkt_array = EGIS0575_GET_CALIBRATION_BYTES_PACKETS_PHASE_3;
      self->pkt_array_len = G_N_ELEMENTS (EGIS0575_GET_CALIBRATION_BYTES_PACKETS_PHASE_3);
      self->current_index = 0;
      child_ssm = fpi_ssm_new (FP_DEVICE (dev), packet_ssm_run_state, PACKET_SSM_DONE);
      fpi_ssm_start_subsm (ssm, child_ssm);
      break;

    case SSM_PRE_CALIBRATION_BYTES_PHASE_4_REQ:
      transfer = fpi_usb_transfer_new (dev);

      fpi_usb_transfer_fill_bulk_full (transfer, EGIS0575_EPOUT, EGIS0575_GET_CALIBRATION_BYTES_PACKETS_PHASE_4.sequence, EGIS0575_GET_CALIBRATION_BYTES_PACKETS_PHASE_4.length, NULL);

      transfer->ssm = ssm;
      transfer->short_is_error = TRUE;

      fpi_usb_transfer_submit (transfer, EGIS0575_TIMEOUT, NULL, fpi_ssm_usb_transfer_cb, NULL);
      break;

    case SSM_PRE_CALIBRATION_BYTES_PHASE_4_RESP:
      transfer = fpi_usb_transfer_new (dev);

      fpi_usb_transfer_fill_bulk (transfer, EGIS0575_EPIN, EGIS0575_GET_CALIBRATION_BYTES_PACKETS_PHASE_4.response_length);

      transfer->ssm = ssm;
      transfer->short_is_error = TRUE;

      fpi_usb_transfer_submit (transfer, EGIS0575_TIMEOUT, NULL, resp_setup, NULL);
      break;

    case SSM_PRE_CALIBRATION_BYTES_PHASE_5:

      self->pkt_array = EGIS0575_GET_CALIBRATION_BYTES_PACKETS_PHASE_5;
      self->pkt_array_len = G_N_ELEMENTS (EGIS0575_GET_CALIBRATION_BYTES_PACKETS_PHASE_5);
      self->current_index = 0;
      child_ssm = fpi_ssm_new (FP_DEVICE (dev), packet_ssm_run_state, PACKET_SSM_DONE);
      fpi_ssm_start_subsm (ssm, child_ssm);
      break;

    case SSM_GET_CALIBRATION_BYTES_REQ:
      transfer = fpi_usb_transfer_new (dev);

      fpi_usb_transfer_fill_bulk_full (transfer, EGIS0575_EPOUT, (unsigned char[]){0x45, 0x47, 0x49, 0x53, 0x72, 0x14, 0xec}, 7, NULL);

      transfer->ssm = ssm;
      transfer->short_is_error = TRUE;

      fpi_usb_transfer_submit (transfer, EGIS0575_TIMEOUT, NULL, fpi_ssm_usb_transfer_cb, NULL);
      break;

    case SSM_GET_CALIBRATION_BYTES_RESP:
      transfer = fpi_usb_transfer_new (dev);

      fpi_usb_transfer_fill_bulk_full (transfer, EGIS0575_EPIN, self->calibration_sequence, EGIS0575_IMGSIZE, NULL);

      transfer->ssm = ssm;
      transfer->short_is_error = TRUE;

      fpi_usb_transfer_submit (transfer, EGIS0575_TIMEOUT, NULL, fpi_ssm_usb_transfer_cb, NULL);
      break;

    case SSM_CHECK_CALIBRATION_BYTES:
      /* Because we get the calibration_sequence every time there are cases that it is broken.
         In that case the last (I think up to ~300) bytes are the same. I my testing it is 0x3f or in ascii '?'.
         If the calibration_sequence ends on at least 100 (arbitrary amount) of the same byte, we discard it and
         ask the user to retry in some time. (this should be fixed when storage is implemented)

         It is caused because i was to lazy making a loop that waits for a specific resopnse, see egis0575.h EGIS0575_GET_CALIBRATION_BYTES_PACKETS
       */

      unsigned char last_byte = self->calibration_sequence[EGIS0575_IMGSIZE - 1];
      gboolean cal_broken = TRUE;

      for(int i = EGIS0575_IMGSIZE - 2; i > EGIS0575_IMGSIZE - 100; i--)
        {
          if(self->calibration_sequence[i] != last_byte)
            {
              cal_broken = FALSE;
              break;
            }
        }

      if(cal_broken)
        {
          fp_dbg ("Setup calibration package is broken, please retry");
          g_clear_pointer (&self->calibration_sequence, g_free);

          // The sensor often isn't ready right after the previous session closed, so just run the setup again
          if (self->calibration_retries++ < EGIS0575_CALIBRATION_RETRIES)
            {
              fpi_ssm_jump_to_state_delayed (ssm, SSM_SETUP_START, EGIS0575_CALIBRATION_RETRY_DELAY);
              return;
            }
          self->calibration_retries = 0;

          fpi_ssm_mark_failed (ssm, g_error_new_literal (G_IO_ERROR, G_IO_ERROR_CANCELLED, "Setup calibration package broken, retry later"));
          return;
        }

      self->calibration_retries = 0;
      fpi_ssm_mark_completed (ssm);
      break;
    }
}

static void
init_ssm_run_state (FpiSsm *ssm, FpDevice *dev)
{
  fpi_ssm_silence_debug (ssm);
  FpDeviceEgis0575 *self = FPI_DEVICE_EGIS0575 (dev);
  FpiUsbTransfer *transfer;
  FpiSsm *child_ssm;

  switch(fpi_ssm_get_cur_state (ssm))
    {
    case SSM_PRE_RESET:
      self->pkt_array = EGIS0575_PRE_RESET_PACKETS;
      self->pkt_array_len = G_N_ELEMENTS (EGIS0575_PRE_RESET_PACKETS);
      self->current_index = 0;
      child_ssm = fpi_ssm_new (FP_DEVICE (dev), packet_ssm_run_state, PACKET_SSM_DONE);
      fpi_ssm_start_subsm (ssm, child_ssm);
      break;

    case SSM_RESET_REQ:
      transfer = fpi_usb_transfer_new (dev);

      fpi_usb_transfer_fill_bulk_full (transfer, EGIS0575_EPOUT, (unsigned char[]){0x45, 0x47, 0x49, 0x53, 0x60, 0x00}, 6, NULL);

      transfer->ssm = ssm;
      transfer->short_is_error = TRUE;

      fpi_usb_transfer_submit (transfer, EGIS0575_TIMEOUT, NULL, fpi_ssm_usb_transfer_cb, NULL);

      break;

    case SSM_RESET_RESP:
      transfer = fpi_usb_transfer_new (dev);

      fpi_usb_transfer_fill_bulk (transfer, EGIS0575_EPIN, 7);

      transfer->ssm = ssm;
      transfer->short_is_error = TRUE;

      fpi_usb_transfer_submit (transfer, EGIS0575_TIMEOUT, NULL, resp_init, NULL);
      break;

    case SSM_POST_RESET:
      self->pkt_array = EGIS0575_POST_RESET_PACKETS;
      self->pkt_array_len = G_N_ELEMENTS (EGIS0575_POST_RESET_PACKETS);
      self->current_index = 0;
      child_ssm = fpi_ssm_new (FP_DEVICE (dev), packet_ssm_run_state, PACKET_SSM_DONE);
      fpi_ssm_start_subsm (ssm, child_ssm);
      break;

    case SSM_CALIBRATION_REQ_1:
      transfer = fpi_usb_transfer_new (dev);
      fpi_usb_transfer_fill_bulk_full (transfer, EGIS0575_EPOUT, EGIS0575_CALIBRATION_PACKET_1.sequence, EGIS0575_CALIBRATION_PACKET_1.length, NULL);

      transfer->ssm = ssm;
      transfer->short_is_error = TRUE;

      fpi_usb_transfer_submit (transfer, EGIS0575_TIMEOUT, NULL, fpi_ssm_usb_transfer_cb, NULL);
      break;

    case SSM_CALIBRATION_REQ_2:
      transfer = fpi_usb_transfer_new (dev);

      fpi_usb_transfer_fill_bulk_full (transfer, EGIS0575_EPOUT, self->calibration_sequence, EGIS0575_IMGSIZE, NULL);

      transfer->ssm = ssm;
      transfer->short_is_error = TRUE;

      fpi_usb_transfer_submit (transfer, EGIS0575_TIMEOUT, NULL, fpi_ssm_usb_transfer_cb, NULL);
      break;

    case SSM_CALIBRATION_RESP:
      transfer = fpi_usb_transfer_new (dev);
      fpi_usb_transfer_fill_bulk (transfer, EGIS0575_EPIN, 7);
      transfer->ssm = ssm;
      fpi_usb_transfer_submit (transfer, EGIS0575_TIMEOUT, NULL, fpi_ssm_usb_transfer_cb, NULL);
      break;

    case SSM_POST_CALIBRATION:
      self->pkt_array = EGIS0575_POST_CALIBRATION_PACKETS;
      self->pkt_array_len = G_N_ELEMENTS (EGIS0575_POST_CALIBRATION_PACKETS);
      self->current_index = 0;
      child_ssm = fpi_ssm_new (FP_DEVICE (dev), packet_ssm_run_state, PACKET_SSM_DONE);
      fpi_ssm_start_subsm (ssm, child_ssm);
      break;

    case SSM_POST_REPEAT:
      self->pkt_array = EGIS0575_POST_REPEAT_PACKETS;
      self->pkt_array_len = G_N_ELEMENTS (EGIS0575_POST_REPEAT_PACKETS);
      self->current_index = 0;
      child_ssm = fpi_ssm_new (FP_DEVICE (dev), packet_ssm_run_state, PACKET_SSM_DONE);
      fpi_ssm_start_subsm (ssm, child_ssm);
      break;

    default:
      g_assert_not_reached ();
    }
}

static void
img_ssm_run_state (FpiSsm *ssm, FpDevice *dev)
{
  fpi_ssm_silence_debug (ssm);

  FpDeviceEgis0575 *self = FPI_DEVICE_EGIS0575 (dev);
  FpiUsbTransfer *transfer;
  FpiSsm *child_ssm;

  switch(fpi_ssm_get_cur_state (ssm))
    {
    case IMG_SSM_FINGER_REQ:
      if(self->stop)
        {
          fpi_ssm_mark_completed (ssm); // loop_complete finishes the deactivation
          return;
        }

      transfer = fpi_usb_transfer_new (dev);
      fpi_usb_transfer_fill_bulk_full (transfer, EGIS0575_EPOUT, (unsigned char[]){0x45, 0x47, 0x49, 0x53, 0x60, 0x01}, 6, NULL);
      transfer->ssm = ssm;
      transfer->short_is_error = TRUE;
      fpi_usb_transfer_submit (transfer, EGIS0575_TIMEOUT, NULL, fpi_ssm_usb_transfer_cb, NULL);
      break;

    case IMG_SSM_FINGER_RESP:
      transfer = fpi_usb_transfer_new (dev);
      fpi_usb_transfer_fill_bulk (transfer, EGIS0575_EPIN, 7);
      transfer->ssm = ssm;
      fpi_usb_transfer_submit (transfer, EGIS0575_TIMEOUT, NULL, resp_finger_present, NULL);
      break;

    case IMG_SSM_PRE_FIRST_IMAGE:
      self->pkt_array = EGIS0575_PRE_FIRST_IMAGE_PACKETS;
      self->pkt_array_len = G_N_ELEMENTS (EGIS0575_PRE_FIRST_IMAGE_PACKETS);
      self->current_index = 0;
      child_ssm = fpi_ssm_new (FP_DEVICE (dev), packet_ssm_run_state, PACKET_SSM_DONE);
      fpi_ssm_start_subsm (ssm, child_ssm);
      break;

    case IMG_SSM_FIRST_IMAGE_REQ:
      transfer = fpi_usb_transfer_new (dev);
      fpi_usb_transfer_fill_bulk_full (transfer, EGIS0575_EPOUT, (unsigned char[]){0x45, 0x47, 0x49, 0x53, 0x64, 0x14, 0xec}, 7, NULL);
      transfer->ssm = ssm;
      transfer->short_is_error = TRUE;
      fpi_usb_transfer_submit (transfer, EGIS0575_TIMEOUT, NULL, fpi_ssm_usb_transfer_cb, NULL);
      break;

    case IMG_SSM_FIRST_IMAGE_RESP:
      transfer = fpi_usb_transfer_new (dev);
      fpi_usb_transfer_fill_bulk (transfer, EGIS0575_EPIN, 5356);
      transfer->ssm = ssm;
      fpi_usb_transfer_submit (transfer, EGIS0575_TIMEOUT, NULL, resp_image, NULL);
      break;

    case IMG_SSM_PRE_REPEAT_IMAGE:
      self->pkt_array = EGIS0575_REPEAT_PACKETS;
      self->pkt_array_len = G_N_ELEMENTS (EGIS0575_REPEAT_PACKETS);
      self->current_index = 0;
      child_ssm = fpi_ssm_new (FP_DEVICE (dev), packet_ssm_run_state, PACKET_SSM_DONE);
      fpi_ssm_start_subsm (ssm, child_ssm);
      break;

    case IMG_SSM_REPEAT_IMAGE_REQ:
      transfer = fpi_usb_transfer_new (dev);
      fpi_usb_transfer_fill_bulk_full (transfer, EGIS0575_EPOUT, (unsigned char[]){0x45, 0x47, 0x49, 0x53, 0x64, 0x14, 0xec}, 7, NULL);
      transfer->ssm = ssm;
      transfer->short_is_error = TRUE;
      fpi_usb_transfer_submit (transfer, EGIS0575_TIMEOUT, NULL, fpi_ssm_usb_transfer_cb, NULL);
      break;

    case IMG_SSM_REPEAT_IMAGE_RESP:
      transfer = fpi_usb_transfer_new (dev);
      fpi_usb_transfer_fill_bulk (transfer, EGIS0575_EPIN, 5356);
      transfer->ssm = ssm;
      fpi_usb_transfer_submit (transfer, EGIS0575_TIMEOUT, NULL, resp_image, NULL);
      break;

    case IMG_SSM_POST_REPEAT:
      self->pkt_array = EGIS0575_POST_REPEAT_PACKETS;
      self->pkt_array_len = G_N_ELEMENTS (EGIS0575_POST_REPEAT_PACKETS);
      self->current_index = 0;
      child_ssm = fpi_ssm_new (FP_DEVICE (dev), packet_ssm_run_state, PACKET_SSM_DONE);
      fpi_ssm_start_subsm (ssm, child_ssm);
      break;

    case IMG_SSM_PROCESS_DATA:
      if(self->strips_len != 0) // if nothing in buffer do not process imges
        process_imgs (ssm, dev);
      g_slist_free_full (g_steal_pointer (&self->strips), g_free);
      self->strips_len = 0;
      fpi_ssm_jump_to_state (ssm, IMG_SSM_FINGER_REQ);
      break;

    case IMG_SSM_FREE_DATA:
      g_slist_free_full (g_steal_pointer (&self->strips), g_free);
      self->strips_len = 0;
      fpi_ssm_next_state (ssm); // leave the cleanup state, so loop_complete gets called
      break;

    default:
      g_assert_not_reached ();
    }
}


/*
 * ==================== Top-level command callback & meta-data ====================
 */

static void
dev_init_done (FpiSsm *ssm, FpDevice *dev, GError *error)
{
  FpImageDevice *img_dev = FP_IMAGE_DEVICE (dev);

  fp_dbg ("EGIS INIT DONE");
  fpi_image_device_open_complete (img_dev, error);
}

static void
dev_setup_done (FpiSsm *ssm, FpDevice *dev, GError *error)
{
  FpDeviceEgis0575 *self = FPI_DEVICE_EGIS0575 (dev);

  if(error || !self->calibration_sequence)
    {
      fp_dbg ("Error occurred in setup phase");
      g_clear_pointer (&self->calibration_sequence, g_free);

      fpi_image_device_open_complete (FP_IMAGE_DEVICE (dev), error);
      return;
    }


  fp_dbg ("EGIS SETUP SSM DONE");
  FpiSsm *init_ssm = fpi_ssm_new (FP_DEVICE (dev), init_ssm_run_state, SSM_INIT_DONE);

  fpi_ssm_start (init_ssm, dev_init_done);
}

static void
dev_init (FpImageDevice *dev)
{
  fp_dbg ("EGIS INIT");
  GError *error = NULL;

  g_usb_device_claim_interface (fpi_device_get_usb_device (FP_DEVICE (dev)), 0, 0, &error);

  FpiSsm *setup_ssm = fpi_ssm_new (FP_DEVICE (dev), setup_ssm_run_state, SSM_SETUP_DONE);

  fpi_ssm_start (setup_ssm, dev_setup_done);
}


static void
dev_deinit (FpImageDevice *dev)
{
  fp_dbg ("EGIS DEINIT");
  GError *error = NULL;
  FpDeviceEgis0575 *self = FPI_DEVICE_EGIS0575 (dev);

  if (self->running)
    {
      self->stop = TRUE;
    }
  else
    {
      g_usb_device_release_interface (fpi_device_get_usb_device (FP_DEVICE (dev)), 0, 0, &error);

      fpi_image_device_close_complete (dev, error);
    }
}

static void
dev_stop (FpImageDevice *dev)
{
  fp_dbg ("EGIS STOP");
  FpDeviceEgis0575 *self = FPI_DEVICE_EGIS0575 (dev);

  if (self->running)
    self->stop = TRUE;
  else
    fpi_image_device_deactivate_complete (dev, NULL);
}

static void
dev_start (FpImageDevice *dev)
{
  fp_dbg ("EGIS START");
  FpDeviceEgis0575 *self = FPI_DEVICE_EGIS0575 (dev);

  FpiSsm *ssm = fpi_ssm_new_full (FP_DEVICE (dev), img_ssm_run_state, IMG_SSM_DONE, IMG_SSM_FREE_DATA, "image capture");

  self->stop = FALSE;

  self->running = TRUE;

  fpi_ssm_start (ssm, loop_complete);

  fp_dbg ("EGIS START DONE");
  fpi_image_device_activate_complete (dev, NULL);

}

static const FpIdEntry id_table[] = {
  { .vid = 0x1c7a, .pid = 0x0575, },
  { .vid = 0,      .pid = 0, },
};

static void
fpi_device_egis0575_init (FpDeviceEgis0575 *self)
{
}

static void
fpi_device_egis0575_finalize (GObject *this)
{
  FpDeviceEgis0575 *self = FPI_DEVICE_EGIS0575 (this);

  g_clear_pointer (&self->calibration_sequence, g_free);
}

static void
fpi_device_egis0575_class_init (FpDeviceEgis0575Class *klass)
{
  FpDeviceClass *dev_class = FP_DEVICE_CLASS (klass);
  FpImageDeviceClass *img_class = FP_IMAGE_DEVICE_CLASS (klass);

  dev_class->id = "egis0575";
  dev_class->full_name = "LighTuning Technology Inc. EgisTec EH575";
  dev_class->type = FP_DEVICE_TYPE_USB;
  dev_class->id_table = id_table;
  dev_class->scan_type = FP_SCAN_TYPE_SWIPE;
  dev_class->nr_enroll_stages = 8;       /* small sensor, more swipes cover more of the finger */

  img_class->img_open = dev_init;
  img_class->img_close = dev_deinit;
  img_class->activate = dev_start;
  img_class->deactivate = dev_stop;

  img_class->img_width = EGIS0575_IMGWIDTH;
  img_class->img_height = -1;

  img_class->bz3_threshold = EGIS0575_BZ3_THRESHOLD;

  G_OBJECT_CLASS (klass)->finalize = fpi_device_egis0575_finalize;
}
