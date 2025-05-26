FROM python:3.9 AS base
ARG POETRY_VERSION=2.0.1

RUN mkdir workspace
RUN apt-get update 
RUN pip install -U \
  pip \
  poetry==${POETRY_VERSION} \
  setuptools \
  && poetry config virtualenvs.create false
COPY pyproject.toml workspace
COPY poetry.lock workspace
COPY app workspace/app
COPY .env workspace
COPY run.py workspace
COPY README.md workspace


WORKDIR /workspace
RUN poetry install
CMD ["python", "run.py"]
