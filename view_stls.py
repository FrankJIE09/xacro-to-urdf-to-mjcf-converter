#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
STL文件可视化工具 (使用 trimesh)

该脚本使用 trimesh 库来逐个显示STL文件。
关闭窗口后，脚本会自动加载下一个模型。
"""
#  if_slider_abpart_link.STL
import os
import glob
import trimesh

def display_stls_with_trimesh(directory="mjcf_models/rohand_left"):
    """
    在指定目录中查找所有STL文件并使用 trimesh 逐个显示。
    """
    search_path_upper = os.path.join(directory, '*.STL')
    search_path_lower = os.path.join(directory, '*.stl')
    
    stl_files = sorted(glob.glob(search_path_upper) + glob.glob(search_path_lower))

    if not stl_files:
        print(f"错误: 在目录 '{directory}' 中未找到任何STL文件。")
        return

    print("=" * 60)
    print(f"在 '{directory}' 中找到 {len(stl_files)} 个STL文件。")
    print("将使用 trimesh 逐个显示模型。关闭查看器窗口以查看下一个。")
    print("=" * 60)

    for i, stl_path in enumerate(stl_files):
        filename = os.path.basename(stl_path)
        print(f"[{i+1}/{len(stl_files)}] 正在显示: {filename}")

        try:
            # 加载STL文件
            mesh = trimesh.load(stl_path)
            
            # trimesh.Scene 对象可以包含一个或多个几何体
            scene = trimesh.Scene(mesh)
            
            # show() 方法会打开一个窗口并阻塞执行，直到窗口被关闭
            scene.show()

        except Exception as e:
            print(f"  -> 在显示 {filename} 时发生错误: {e}")

if __name__ == "__main__":
    display_stls_with_trimesh() 