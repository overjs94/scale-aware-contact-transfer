FROM python:3.12-slim-bookworm
RUN apt-get update && apt-get install -y --no-install-recommends \
    git libegl1 libgl1 libglfw3 && rm -rf /var/lib/apt/lists/*
WORKDIR /work
COPY . .
RUN python -m pip install --no-cache-dir -e ".[dev,simulation]"
ENV OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
CMD ["bash"]
