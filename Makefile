NAME=osrf/ros2
VERSION=humble
CONTAINER_NAME=ros2


# command to list all make commands
help:
	@echo -e "\033[1;34m[make targets]:\033[0m"
	@egrep -h '\s##\s' $(MAKEFILE_LIST) \
		| awk 'BEGIN {FS = ":.*?## "}; \
		{printf "\033[1;36m%-20s\033[0m %s\n", $$1, $$2}'


# command to build docker image
build:
	docker build -t $(NAME):$(VERSION) .


# command to create docker container to run simulations and visualisations
run_display:
	@xhost +local:root && docker run \
		-it \
		--rm \
		-e DISPLAY \
		-v /tmp/.X11-unix:/tmp/.X11-unix \
		-e QT_X11_NO_MITSHM=1 \
		--privileged \
		--ipc=host \
		--pid=host \
		--network="host" \
		--name $(CONTAINER_NAME) \
		$(NAME):$(VERSION)

	
container=`docker ps -a -q`
image=`docker images | awk '/^<none>/ { print $$3 }'`


# command to start navigation controller
run_nav:
	docker exec -it $(CONTAINER_NAME) sh -c "ros2 run sam_bot_controller sam_bot_controller_exe"


# command to remove all docker assets 
cleanup:
	@if [ "$(image)" != "" ] ; then \
		echo "Removing Docker image: $(image)"; \
		docker rmi $(image); \
	fi
	@if [ "$(container)" != "" ] ; then \
		echo "Removing Docker container: $(container)"; \
		docker rm $(container); \
	fi
	
