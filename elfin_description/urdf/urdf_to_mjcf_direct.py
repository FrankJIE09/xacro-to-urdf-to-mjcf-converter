#!/usr/bin/env python3
"""
直接的URDF转MJCF转换工具
正确处理mesh文件路径
"""

import mujoco
import os
import sys
import argparse
import shutil
import re
from pathlib import Path

def preprocess_urdf_for_mjcf(urdf_path, output_path):
    """
    预处理URDF文件，将package://路径转换为绝对路径
    """
    try:
        with open(urdf_path, 'r', encoding='utf-8') as f:
            content = f.read()
        
        # 获取工作目录的绝对路径
        base_dir = os.path.abspath(os.path.dirname(urdf_path))
        project_root = os.path.abspath('.')
        
        # 替换package://路径为绝对路径
        def replace_package_path(match):
            package_path = match.group(1)
            # 去掉 package://elfin_description/ 前缀
            if package_path.startswith('elfin_description/'):
                relative_path = package_path[len('elfin_description/'):]
                absolute_path = os.path.join(project_root, 'elfin_description', relative_path)
            elif package_path.startswith('ur_description/'):
                relative_path = package_path[len('ur_description/'):]
                absolute_path = os.path.join(project_root, 'universal_robot', 'ur_description', relative_path)
            else:
                # 其他情况，直接使用项目根目录
                absolute_path = os.path.join(project_root, package_path)
            
            return f'{absolute_path}'
        
        # 使用正则表达式替换package://路径
        pattern = r'package://([^"]+)'
        new_content = re.sub(pattern, replace_package_path, content)
        
        # 确保输出目录存在
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        
        # 写入处理后的文件
        with open(output_path, 'w', encoding='utf-8') as f:
            f.write(new_content)
        
        print(f"✅ 预处理完成: {output_path}")
        return True
        
    except Exception as e:
        print(f"❌ 预处理失败: {e}")
        return False

def convert_urdf_to_mjcf(urdf_path, output_path):
    """
    转换URDF文件到MJCF格式
    """
    try:
        print(f"🔄 开始转换: {urdf_path}")
        
        # 先预处理URDF文件
        temp_urdf = output_path.replace('.xml', '_temp.urdf')
        if not preprocess_urdf_for_mjcf(urdf_path, temp_urdf):
            return False
        
        print(f"🔄 加载处理后的URDF文件...")
        
        # 保存当前工作目录
        original_cwd = os.getcwd()
        
        try:
            # 切换到URDF文件所在目录，确保相对路径正确
            urdf_dir = os.path.dirname(os.path.abspath(urdf_path))
            os.chdir(urdf_dir)
            
            # 加载预处理后的URDF文件
            model = mujoco.MjModel.from_xml_path(os.path.abspath(temp_urdf))
        finally:
            # 恢复原工作目录
            os.chdir(original_cwd)
        
        print(f"✅ URDF加载成功")
        print(f"   - 关节数: {model.njnt}")
        print(f"   - 连杆数: {model.nbody}")
        print(f"   - 几何体数: {model.ngeom}")
        
        # 保存为MJCF文件
        print(f"🔄 保存MJCF文件: {output_path}")
        mujoco.mj_saveLastXML(output_path, model)
        
        # 保留临时文件用于调试
        # if os.path.exists(temp_urdf):
        #     os.remove(temp_urdf)
        
        # 检查输出文件
        if os.path.exists(output_path):
            file_size = os.path.getsize(output_path)
            print(f"✅ 转换成功!")
            print(f"   - 输出文件: {output_path}")
            print(f"   - 文件大小: {file_size} bytes")
            return True
        else:
            print("❌ 输出文件未生成")
            return False
            
    except Exception as e:
        print(f"❌ 转换失败: {e}")
        # 保留临时文件用于调试
        # temp_urdf = output_path.replace('.xml', '_temp.urdf')
        # if os.path.exists(temp_urdf):
        #     os.remove(temp_urdf)
        return False

def batch_convert():
    """批量转换所有URDF文件"""
    urdf_files = []
    
    # 查找Elfin URDF文件
    elfin_dir = "elfin_description/urdf"
    if os.path.isdir(elfin_dir):
        for file in os.listdir(elfin_dir):
            if file.endswith('.urdf'):
                urdf_files.append((os.path.join(elfin_dir, file), 'elfin'))
    
    # 查找UR URDF文件
    ur_dir = "universal_robot/ur_description/urdf"
    if os.path.isdir(ur_dir):
        for file in os.listdir(ur_dir):
            if file.endswith('.urdf'):
                urdf_files.append((os.path.join(ur_dir, file), 'ur'))
    
    if not urdf_files:
        print("❌ 未找到URDF文件")
        return
    
    print(f"🤖 找到 {len(urdf_files)} 个URDF文件")
    print("="*60)
    
    success_count = 0
    for urdf_file, robot_type in urdf_files:
        print(f"\n🔄 转换 [{robot_type}]: {urdf_file}")
        
        basename = os.path.splitext(os.path.basename(urdf_file))[0]
        output_file = f"mjcf_models/{basename}.xml"
        
        if convert_urdf_to_mjcf(urdf_file, output_file):
            success_count += 1
        
        print("-"*60)
    
    print(f"\n✅ 批量转换完成: {success_count}/{len(urdf_files)} 成功")
    
    if success_count > 0:
        print(f"📂 生成的MJCF文件位于: mjcf_models/")
        print("💡 可以使用以下命令查看MJCF文件:")
        print("   python mjcf_viewer.py mjcf_models/elfin3.xml")

def main():
    parser = argparse.ArgumentParser(description='直接的URDF转MJCF工具')
    parser.add_argument('urdf_file', nargs='?', help='输入URDF文件路径')
    parser.add_argument('-o', '--output', help='输出MJCF文件路径')
    parser.add_argument('--batch', action='store_true', help='批量转换模式')
    
    args = parser.parse_args()
    
    if args.batch:
        batch_convert()
    elif args.urdf_file:
        urdf_file = args.urdf_file
        
        if not os.path.exists(urdf_file):
            print(f"❌ URDF文件不存在: {urdf_file}")
            sys.exit(1)
        
        # 确定输出文件路径
        if args.output:
            output_file = args.output
        else:
            basename = os.path.splitext(os.path.basename(urdf_file))[0]
            output_file = f"mjcf_models/{basename}.xml"
        
        # 确保输出目录存在
        os.makedirs(os.path.dirname(output_file), exist_ok=True)
        
        success = convert_urdf_to_mjcf(urdf_file, output_file)
        sys.exit(0 if success else 1)
    else:
        parser.print_help()
        sys.exit(1)

if __name__ == '__main__':
    main() 