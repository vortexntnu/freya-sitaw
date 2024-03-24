# Freya SITAW
Collection of launch files and specific versions of repositories that work together.

## Dependencies
- [vortex-msgs]()
- [vortex-vkf]()
- [vortex-sensor-fusion]()

## Usage
Add a submodule:
```bash
cd packages
git submodule add <repository-url>
```

Update submodules:
```bash
git submodule update --init --recursive
```

Set branch of submodule:
```bash
cd <packages/name-of-submodule>
git checkout <branch>
```
