#!/bin/bash
echo "Downloading weights archive"
wget --load-cookies /tmp/cookies.txt "https://docs.google.com/uc?export=download&confirm=$(wget --quiet --save-cookies /tmp/cookies.txt --keep-session-cookies --no-check-certificate 'https://docs.google.com/uc?export=download&id=1Miq60F3vctwarW-M-lS-sZlypCj1Ly78' -O- | sed -rn 's/.*confirm=([0-9A-Za-z_]+).*/\1\n/p')&id=1Miq60F3vctwarW-M-lS-sZlypCj1Ly78" -O temp.tar.xz && rm -rf /tmp/cookies.txt
echo "Extracting weights files"
tar -xf temp.tar.xz
echo "Removing temporary archive"
rm temp.tar.xz
