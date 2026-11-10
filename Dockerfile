FROM python:3.12-slim-bookworm
RUN apt-get update && apt-get install -y --no-install-recommends \
    git nodejs npm libegl1 libgl1 libglfw3 && rm -rf /var/lib/apt/lists/*
WORKDIR /work
COPY . .
RUN python -m pip install --no-cache-dir -e ".[dev,simulation]"
RUN npm --prefix web ci && npm --prefix web run build
ENV OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
EXPOSE 8000
CMD ["bash"]
