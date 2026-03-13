import os

# i love shoving types into an untyped language
from typing import Callable, Optional, TypeVar, cast

import docker
from docker.errors import DockerException, NotFound
from docker.models.containers import Container
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel


app = FastAPI(title="rosbag api")

app.add_middleware(
    CORSMiddleware,
    # i use cors btw
    allow_origins=["*"],
    allow_credentials=False,
    allow_methods=["*"],
    allow_headers=["*"],
)


class HealthResponse(BaseModel):
    ok: bool = True


class ContainerStatus(BaseModel):
    id: str
    name: str
    status: str


class BagActionResponse(ContainerStatus):
    action: str


# gets the docker client
def _docker_client() -> docker.DockerClient:
    base_url = os.environ.get("DOCKER_HOST", "unix://var/run/docker.sock")
    return docker.DockerClient(base_url=base_url)


def _find_rosbag_container(client: docker.DockerClient) -> Container:
    name = os.environ.get("ROSBAG_CONTAINER_NAME", "rosbag")
    try:
        return cast(Container, client.containers.get(name))
    except NotFound as exc:
        raise HTTPException(
            status_code=404, detail=f"container not found: {name}"
        ) from exc


T = TypeVar("T")


def _with_client(fn: Callable[[docker.DockerClient], T]) -> T:
    client = _docker_client()
    try:
        return fn(client)
    except DockerException as exc:
        raise HTTPException(status_code=500, detail=f"docker error: {exc}") from exc
    finally:
        try:
            client.close()
        except Exception:
            pass


def _container_status(container: Container) -> ContainerStatus:
    container.reload()
    return ContainerStatus(
        id=cast(str, container.id),
        name=cast(str, container.name),
        status=cast(str, container.status),
    )


@app.get("/healthz", response_model=HealthResponse)
def healthz() -> HealthResponse:
    return HealthResponse()


@app.get("/bag/status", response_model=ContainerStatus)
def bag_status() -> ContainerStatus:
    def _handler(client: docker.DockerClient) -> ContainerStatus:
        container = _find_rosbag_container(client)
        return _container_status(container)

    return _with_client(_handler)


@app.post("/bag/start", response_model=BagActionResponse)
def bag_start() -> BagActionResponse:
    def _handler(client: docker.DockerClient) -> BagActionResponse:
        container = _find_rosbag_container(client)
        container.reload()
        # already was running, no need to do anything
        if container.status == "running":
            status = _container_status(container)
            return BagActionResponse(**status.model_dump(), action="noop")
        container.start()
        status = _container_status(container)
        return BagActionResponse(**status.model_dump(), action="started")

    return _with_client(_handler)


@app.post("/bag/stop", response_model=BagActionResponse)
def bag_stop() -> BagActionResponse:
    def _handler(client: docker.DockerClient) -> BagActionResponse:
        container = _find_rosbag_container(client)
        container.reload()
        # was already not running, ie stopped, so no need to do anything
        if container.status != "running":
            status = _container_status(container)
            return BagActionResponse(**status.model_dump(), action="noop")
        # 20 second timeout should be more than enough
        container.stop(timeout=20)
        status = _container_status(container)
        return BagActionResponse(**status.model_dump(), action="stopped")

    return _with_client(_handler)
