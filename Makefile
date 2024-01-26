NAME=osrf/ros2
VERSION=humble
CONTAINER_NAME=ros2


help:
	@echo "Available targets:"
	@echo "  - build:   		Create docker image"
	@echo "  - run_display:     Runs docker container and launches simulations and visualisations"
	@echo "  - run_nav:   		Creates bash process in container to control robot navigation"
	@echo "  - run_nav:   		Removes image and container"
	@echo "  - help:    		Show this help message"


build:
	docker build -t $(NAME):$(VERSION) .


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


run_nav:
	docker exec -it $(CONTAINER_NAME) sh -c "ros2 run sam_bot_controller sam_bot_controller_exe"


cleanup:
	@if [ "$(shell docker images -q $(NAME):$(VERSION))" != "" ] ; then \
		echo "Removing Docker image: $(NAME):$(VERSION)"; \
		docker rmi $(NAME):$(VERSION); \
	fi
	@if [ "$(shell docker ps -a -q -f name=$(CONTAINER_NAME))" != "" ] ; then \
		echo "Removing Docker container: $(CONTAINER_NAME)"; \
		docker rm $(CONTAINER_NAME); \
	fi
