docker-build:
	docker build -t rviz-detections:latest .

docker-launch:
	docker run \
	-d \
	--rm \
	-v `pwd`/launch/rviz_detections.launch:/app/install/share/rviz_detections/rviz_detections.launch \
	--network=host \
	rviz-detections:latest
