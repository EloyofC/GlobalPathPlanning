#!/bin/sh

./test >out2
if ! cmp -s out1 out2
then
    echo BAD
fi
