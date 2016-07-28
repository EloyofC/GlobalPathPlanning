#!/bin/sh

./test >out2
if ! cmp -s out1 out2
then
    diff out1 out2
fi
