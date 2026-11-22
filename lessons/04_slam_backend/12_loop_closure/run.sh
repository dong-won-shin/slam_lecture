#!/bin/bash
set -eu
cd "$(dirname "$0")/build"

./feature_training ../data/
./loop_closure ../data/vocab_larger.yml.gz ../data/
