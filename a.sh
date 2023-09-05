fetch https://github.com/alastorid/if_ixv/archive/refs/heads/main.zip
tar -xvf main.zip
cd if_ixv-main/src
sed -i .bak '/DIXGBE_DEBUG/ s/#//' Makefile
make
make install
