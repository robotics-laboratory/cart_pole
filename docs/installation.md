# Quick Install Guide

1. Clone the repository
```
git clone https://github.com/dasimagin/cart_pole && cd cart_pole
```
2. Install python requirements
```
python -m pip install -r requirements.txt
```
3. Install [pre-commit hooks](https://pre-commit.com/)
```
pre-commit install
```
4. Run tests
```
pytest
```

## Dangerous hooks

In order to run hooks that modify files (black & isort formatters):

```
pre-commit run --hook-stage manual --all-files
```

You can also run it for one file, or create a file watcher:

```
pre-commit run --hook-stage manual --files <file-to-check>
```
