## Kaldi-gstreamer-server

1. Build the image
```
docker build -t kaldi-gstreamer-server:1.0 https://github.com/jcsilva/docker-kaldi-gstreamer-server
```
Or simply pull:
```
docker pull jcsilva/docker-kaldi-gstreamer-server
```
2. Confirm that in the `models/asr/sample_worker.yaml` the directories begin with /opt/models
3. Run the image
```
docker run -tid --name kaldigstreamer -p 8080:80 -v <absolute-path-to-models/asr>:/opt/models jcsilva/docker-kaldi-gstreamer-server:latest
```
4. Inside the container run `/opt/start.sh -y /opt/models/sample_worker.yaml`
5. Run the asr server and then the client to send a request to the service
6. To stop run `/opt/stop.sh`
7. To delete the docker process entirely run `docker rm -f kaldigstreamer`

## Docker image (English model)

### Build:

docker pull jcsilva/docker-kaldi-gstreamer-server:tedlium

### Run-and-Play

docker run -p 8080:80 jcsilva/docker-kaldi-gstreamer-server:tedlium