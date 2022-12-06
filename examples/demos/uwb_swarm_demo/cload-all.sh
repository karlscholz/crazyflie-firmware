#!/usr/bin/env bash

for id in {0..3}
do
echo "Flasing $id"
python.exe -m cfloader flash build/cf2.bin stm32-fw -w radio://0/80/2M/E7E7E7E7E$id
done