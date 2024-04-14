# Freya SITAW
Collection of launch files and specific versions of repositories that work together on the xavier of Freya.

## Dependencies
- [ouster-lidar-ros2-driver](https://github.com/vortexntnu/ouster-lidar-ros2-driver)
- [vortex-msgs](https://github.com/vortexntnu/vortex-msgs)
- [vortex-sensor-fusion](https://github.com/vortexntnu/vortex-sensor-fusion)
- [vortex-vkf](https://github.com/vortexntnu/vortex-vkf)
- [zed-ros2-wrapper](https://github.com/vortexntnu/zed-ros2-wrapper)

## How to work with submodules
A git submodule is nothing more than a repository inside another repository. The submodules are stored as links to specific commits of their respective repositories.

You need to clone this repository with the `--recursive` option 

### Commands
Add a new submodule (repository):
```bash
cd packages
git submodule add <repository-url>
```

Update submodules if they are not cloned:
```bash
git submodule update --init --recursive
```

Update submodules (when you switch branches for example)
```bash
git submodule update
``` 

If you switch to a branch that has a different set of submodules, you need to clean the git repository:
```bash
git clean -f -f -d
```
Be careful with this command as this will also remove all untracked changes.