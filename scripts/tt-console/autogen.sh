(cd ../../../modules/libjaylink; ./autogen.sh)

libtoolize --automake --copy
aclocal
autoheader
autoconf
automake --add-missing
