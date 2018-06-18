mkdir drake_binary
cd drake_binary
curl -o drake.tar.gz https://drake-packages.csail.mit.edu/drake/continuous/drake-latest-mac.tar.gz
sleep 30
sudo tar -xvzf drake.tar.gz -C /opt
