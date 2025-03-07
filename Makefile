# Default container name if not provide
CONTAINER_NAME ?= ws_anscer
IMAGE_NAME ?= ws_anscer
TAG:=latest

.PHONY: all build up clean exec stop

all: build up

# Target to build the Docker image
build:
	@echo "Building Docker image..."
	docker compose build 

# Target to run the Docker container
up: 	
	docker compose up --detach --no-build $(IMAGE_NAME) 

# Getting inside the container
exec: 
	docker exec -it $(CONTAINER_NAME) zsh

# Target to stop and remove the Docker container
clean: 
	@echo "Stopping and removing Docker container with name $(CONTAINER_NAME)..."
	docker stop $(CONTAINER_NAME) || true
	docker rm $(CONTAINER_NAME) || true
	@echo "Removing Docker image..."
	docker rmi $(IMAGE_NAME):$(TAG) || true 

stop:
	docker stop $(CONTAINER_NAME) || true
