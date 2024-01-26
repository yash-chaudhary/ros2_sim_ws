NAME=osrf/ros2
VERSION=humble
CONTAINER_NAME=ros2


help:
	@printf "\nUsage: make <command>\n"
	@grep -F -h "##@" $(MAKEFILE_LIST) | grep -F -v grep -F | sed -e 's/\\$$//' | awk 'BEGIN {FS = ":*[[:space:]]*##@[[:space:]]*"}; \
	{ \
		if($$2 == "") \
			pass; \
		else if($$0 ~ /^#/) \
			printf "\n%s\n", $$2; \
		else if($$1 == "") \
			printf "     %-20s%s\n", "", $$2; \
		else \
			printf "\n    \033[34m%-20s\033[0m %s\n", $$1, $$2; \
	}'


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
