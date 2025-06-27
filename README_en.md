# Reliable URDF to MJCF Converter

This is a tool for batch converting URDF (Unified Robot Description Format) files to MJCF (MuJoCo Physics Engine XML format).
This project aims to solve common failures during the conversion process caused by issues with mesh file paths, providing a stable and reliable conversion workflow.
It currently supports Elfin and UR series robots, but can be easily extended to support other robots.

## ✨ Key Features

- **Reliable Path Handling**: Greatly improves conversion success rate by intelligently handling mesh file paths, ensuring the MuJoCo compiler can correctly locate all resources.
- **Batch Conversion**: Automatically discovers and converts all URDF files in specified directories.
- **Self-Contained Output**: Creates a separate folder for each robot model, containing the converted MJCF file and all associated mesh files for easy distribution and use.
- **Easy to Extend**: The code is well-structured, making it easy to add support for new types of robots.
- **Visualization Support**: Includes a simple MJCF viewer (`mjcf_viewer.py`) for quick verification of the conversion results.

## 📂 Project Structure

```
.
├── mjcf_models/            # Stores the output MJCF models after conversion
├── elfin_description/      # Contains URDF and mesh files for Elfin series robots
├── ur_description/         # Contains URDF and mesh files for UR series robots
├── urdf_converter.py       # The core conversion script
├── mjcf_viewer.py          # MJCF model viewer
├── requirements.txt        # Python dependencies
└── README.md
```

## 🚀 Dependencies

Ensure that the necessary libraries are installed in your Python environment.

1.  **Install Dependencies**
    ```bash
    pip install -r requirements.txt
    ```
    The `requirements.txt` file includes the following core libraries:
    - `mujoco`: The core physics engine and conversion tool.
    - `numpy`: A library for scientific computing.

## 💡 How to Use

### 1. Prepare URDF Files

Place your robot description files (URDF and meshes) into the appropriate directories.

- For Elfin robots, place them in the `elfin_description` directory.
- For UR robots, place them in the `ur_description` directory.

> **Note**: If your model is in `.xacro` format, you need to convert it to `.urdf` first. You can refer to the examples in the `xacro_to_urdf.sh` script for this step. This is typically done using the `xacro` command:
> ```bash
> xacro input.xacro > output.urdf
> ```

### 2. Run the Conversion

Execute the core conversion script to start the batch conversion.

```bash
python urdf_converter.py
```

The script will automatically scan for all `.urdf` files in the `elfin_description/urdf` and `ur_description/urdf` directories and perform the conversion.
Upon successful conversion, the results will be saved in the `mjcf_models/` directory. Each model will have its own subdirectory, for example, `mjcf_models/elfin5/`.

### 3. View the Model

Use the provided `mjcf_viewer.py` script to visualize and verify the converted model.

```bash
python mjcf_viewer.py mjcf_models/elfin5/elfin5_with_sphere.xml
```

For example, to view the `elfin5` model:

```bash
python mjcf_viewer.py mjcf_models/elfin5/elfin5_with_sphere.xml
```

## 🧰 Auxiliary Scripts

This project includes several useful auxiliary scripts to streamline the development process.

- **`xacro_to_urdf.sh`**:
  - **Function**: Converts `.xacro` files to `.urdf` files, either individually or in batches. This is an important preprocessing step before running the main `urdf_converter.py` script.
  - **Dependencies**: Requires a ROS environment and the `xacro` package.
  - **Usage**:
    ```bash
    # Convert a single model (e.g., elfin5)
    ./xacro_to_urdf.sh elfin5

    # Use interactive mode to select a model
    ./xacro_to_urdf.sh
    ```

- **`view_in_rviz.sh`**:
  - **Function**: A one-click script to visualize the robot model in ROS's **RViz**. It automatically handles the XACRO to URDF conversion and launches RViz.
  - **Dependencies**: Requires a full ROS environment (including RViz and roslaunch).
  - **Usage**:
    ```bash
    # View the elfin5 model in RViz
    ./view_in_rviz.sh elfin5
    ```

- **`generate_requirements.sh`**:
  - **Function**: Uses `pipreqs` to automatically scan the project and generate a `requirements.txt` file, while also injecting current environment information at the top of the file.
  - **Dependencies**: `pipreqs` (the script will attempt to install it if not found).
  - **Usage**:
    ```bash
    ./generate_requirements.sh
    ```

## 📝 Notes

- **Mesh File Formats**: The script currently supports `.stl`, `.dae`, `.obj`, and `.ply` mesh file formats.
- **ROS Environment**: The `view_in_rviz.sh` and `xacro_to_urdf.sh` scripts are intended for use in a ROS environment.
- **Custom Robots**: To add support for a new robot, you will need to modify the `urdf_converter.py` script:
    1.  Add logic for the new robot type in the `convert_robot` function, specifying the source directories for URDF and mesh files.
    2.  Update the logic for generating the `robots_to_convert` list in the `main` function.

---
*This README was generated by an AI programming assistant.* 