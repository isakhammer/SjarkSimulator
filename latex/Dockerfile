FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
WORKDIR /workspace

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
      make \
      latexmk \
      texlive-latex-base \
      texlive-latex-recommended \
      texlive-latex-extra \
      texlive-pictures \
      texlive-fonts-recommended \
      texlive-science \
    && rm -rf /var/lib/apt/lists/*

CMD ["make", "-C", "latex", "pdf"]

