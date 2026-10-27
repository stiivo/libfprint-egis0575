#!/bin/bash
# Builds the patched libfprint RPM for Fedora in ~/rpmbuild
set -euo pipefail

here="$(cd "$(dirname "$0")" && pwd)"
version="$(rpmspec -q --qf '%{version}\n' "$here/libfprint.spec" | head -1)"

mkdir -p ~/rpmbuild/SOURCES ~/rpmbuild/SPECS
cp "$here/libfprint.spec" ~/rpmbuild/SPECS/
cp "$here/libfprint-egis0575.patch" ~/rpmbuild/SOURCES/
curl -fL -o ~/rpmbuild/SOURCES/libfprint-v$version.tar.gz \
  "https://gitlab.freedesktop.org/libfprint/libfprint/-/archive/v$version/libfprint-v$version.tar.gz"

sudo dnf builddep -y ~/rpmbuild/SPECS/libfprint.spec
rpmbuild -bb ~/rpmbuild/SPECS/libfprint.spec

echo
echo "Built packages:"
ls ~/rpmbuild/RPMS/*/libfprint-$version-*egis0575*.rpm
