#!/bin/bash

OUTPUT_DIR=$1
MEASUREMENT_NAME=$2

mkdir -p $OUTPUT_DIR/$MEASUREMENT_NAME
cp $masterdir/config_settings/* $OUTPUT_DIR/$MEASUREMENT_NAME
