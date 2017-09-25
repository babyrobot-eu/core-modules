## Kaldi-gstreamer-server

1. Build the image
```
docker build -t kaldi-gstreamer-server:1.0 https://github.com/jcsilva/docker-kaldi-gstreamer-server
```
2. Run the image
```
docker run -tid -p 8080:80 -v <kaldi-model-dir>:/opt/models jcsilva/docker-kaldi-gstreamer-server:latest
```
3. Inside the container run `/opt/start.sh -y /opt/models/nnet2.yaml`
4. Use the client to send a request to the service
5. To stop run `/opt/stop.sh`

## Docker image (English model)

### Build:

docker pull jcsilva/docker-kaldi-gstreamer-server:tedlium

### Run-and-Play

docker run -p 8080:80 jcsilva/docker-kaldi-gstreamer-server:tedlium