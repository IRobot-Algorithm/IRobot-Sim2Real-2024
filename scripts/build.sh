#!/bin/bash
CLIENT_IMAGE=${CLIENT_IMAGE:-client-custom:latest}
docker build . -t $CLIENT_IMAGE