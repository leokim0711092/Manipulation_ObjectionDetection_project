# Zenoh-Pointcloud

Zenoh implementation to connect a depth camera (pointcloud messages) to a remote rosject.

- To connect, zenoh must be installed. Use `install_zenoh.sh`.

Two scripts in `init/` are used to connect camera to rosject. To run:

- In host: `./zenoh_pointcloud_local.sh`.
- In rosject `./zenoh_pointclud_rosject.sh`.

Specific ROS 2 topics can be allowed/denied in `.json5` config files inside `config/zenoh-plugin-dds/`.

If The Construct's office public IP changes, it must be changed in `config/router/rosject_config.json5`.
