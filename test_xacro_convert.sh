#!/bin/bash

# ==============================================================================
# 简单的XACRO转URDF测试脚本
#
# 功能: 快速测试xacro到urdf的转换功能，支持Elfin和UR机器人系列
# 用法: ./test_xacro_convert.sh [model_name]
# 示例: 
#   ./test_xacro_convert.sh elfin3          # 转换elfin3模型
#   ./test_xacro_convert.sh elfin:elfin5    # 使用完整格式转换elfin5
#   ./test_xacro_convert.sh ur:ur3          # 转换UR3机器人
#   ./test_xacro_convert.sh                 # 交互式选择模型
#
# 作者: AI Assistant
# 版本: 2.0
# 更新日期: $(date +%Y-%m-%d)
# 
# 支持的机器人类型:
#   - Elfin系列: elfin3, elfin5, elfin10, elfin15, elfin5_l, elfin10_l
#   - UR系列: ur3, ur5, ur10, ur3e, ur5e, ur10e等
# ==============================================================================

# 严格模式：遇到错误立即退出
set -e

# ============================================================================== 
# 颜色定义和常量配置
# ==============================================================================

# ANSI颜色代码定义，用于美化终端输出
GREEN='\033[0;32m'    # 绿色：成功信息
RED='\033[0;31m'      # 红色：错误信息  
BLUE='\033[0;34m'     # 蓝色：普通信息
NC='\033[0m'          # 重置颜色

# ============================================================================== 
# 目录路径配置
# ==============================================================================

# 获取脚本所在目录的绝对路径
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Elfin机器人相关目录
URDF_DIR="${SCRIPT_DIR}/elfin_description/urdf"       # Elfin机器人URDF文件目录

# UR机器人相关目录  
UR_URDF_DIR="${SCRIPT_DIR}/ur_description/urdf"  # UR机器人URDF文件目录

# ============================================================================== 
# 打印函数：统一的信息输出格式
# ==============================================================================

# 打印蓝色信息消息
print_info() {
    echo -e "${BLUE}ℹ️  $1${NC}"
}

# 打印绿色成功消息
print_success() {
    echo -e "${GREEN}✅ $1${NC}"
}

# 打印红色错误消息
print_error() {
    echo -e "${RED}❌ $1${NC}"
}

# ============================================================================== 
# 模型发现和列表功能
# ==============================================================================

# 函数: list_models
# 功能: 扫描并列出所有可用的机器人模型
# 说明: 自动检测elfin_description和universal_robot目录中的XACRO文件
list_models() {
    print_info "可用的机器人模型:"
    
    # ========== Elfin机器人检测 ==========
    echo "  📁 Elfin 机器人:"
    # 遍历elfin_description/urdf目录中的所有.urdf.xacro文件
    for xacro_file in "${URDF_DIR}"/*.urdf.xacro; do
        # 检查文件是否真实存在（避免通配符匹配失败的情况）
        if [ -f "$xacro_file" ]; then
            # 提取模型名称（去掉路径和扩展名）
            model_name=$(basename "$xacro_file" .urdf.xacro)
            echo "    ✅ elfin:$model_name"
        fi
    done
    
    # ========== UR机器人检测 ==========
    # 首先检查UR描述目录是否存在
    if [ -d "$UR_URDF_DIR" ]; then
        echo "  📁 UR (Universal Robot) 机器人:"
        # 遍历universal_robot/ur_description/urdf目录中的所有.xacro文件
        for xacro_file in "${UR_URDF_DIR}"/*.xacro; do
            # 检查文件存在且不是通用的ur.xacro文件（这是模板文件，不是具体模型）
            if [ -f "$xacro_file" ] && [ "$(basename "$xacro_file")" != "ur.xacro" ]; then
                # 提取模型名称
                model_name=$(basename "$xacro_file" .xacro)
                echo "    ✅ ur:$model_name"
            fi
        done
    fi
}

# ============================================================================== 
# XACRO到URDF转换核心功能
# ==============================================================================

# 函数: convert_model
# 参数: $1 - 模型输入字符串（支持"type:name"格式或单独名称）
# 功能: 将指定的XACRO文件转换为URDF文件
# 返回: 0表示成功，1表示失败
convert_model() {
    local model_input="$1"    # 用户输入的模型标识符
    local model_type=""       # 机器人类型（elfin或ur）
    local model_name=""       # 具体的模型名称
    local xacro_file=""       # 源XACRO文件路径
    local urdf_file=""        # 目标URDF文件路径
    
    # ========== 输入解析 ==========
    # 解析模型输入格式，支持两种格式：
    # 1. "type:name" 格式 (如: elfin:elfin3, ur:ur5)
    # 2. 单独名称格式 (如: elfin3) - 默认为elfin类型
    if [[ "$model_input" == *":"* ]]; then
        # 使用冒号分割字符串
        model_type=$(echo "$model_input" | cut -d':' -f1)  # 提取类型
        model_name=$(echo "$model_input" | cut -d':' -f2)  # 提取名称
    else
        # 向后兼容：如果没有指定类型，默认为elfin
        model_type="elfin"
        model_name="$model_input"
    fi
    
    # ========== 文件路径确定 ==========
    # 根据机器人类型确定相应的文件路径
    if [ "$model_type" = "elfin" ]; then
        # Elfin机器人：使用.urdf.xacro扩展名
        xacro_file="${URDF_DIR}/${model_name}.urdf.xacro"
        urdf_file="${URDF_DIR}/${model_name}.urdf"
    elif [ "$model_type" = "ur" ]; then
        # UR机器人：使用.xacro扩展名
        xacro_file="${UR_URDF_DIR}/${model_name}.xacro"
        urdf_file="${UR_URDF_DIR}/${model_name}.urdf"
    else
        # 不支持的机器人类型
        print_error "不支持的机器人类型: $model_type"
        print_info "支持的类型: elfin, ur"
        return 1
    fi
    
    # ========== 文件存在性检查 ==========
    # 检查源XACRO文件是否存在
    if [ ! -f "$xacro_file" ]; then
        print_error "XACRO文件不存在: $xacro_file"
        print_info "请检查模型名称是否正确，或运行 $0 --list 查看可用模型"
        return 1
    fi
    
    print_info "转换 ${model_type}:${model_name} ..."
    print_info "源文件: $xacro_file"
    print_info "目标文件: $urdf_file"
    
    # ========== 依赖检查 ==========
    # 检查xacro命令是否可用
    if ! command -v xacro &> /dev/null; then
        print_error "xacro命令未找到，请安装: sudo apt install ros-\$ROS_DISTRO-xacro"
        print_info "或确保ROS环境已正确设置: source /opt/ros/\$ROS_DISTRO/setup.bash"
        return 1
    fi
    
    # ========== 环境设置 ==========
    # 设置ROS包路径，确保xacro能找到依赖的包
    export ROS_PACKAGE_PATH="$SCRIPT_DIR:${ROS_PACKAGE_PATH:-}"
    print_info "ROS包路径: $ROS_PACKAGE_PATH"
    
    # ========== 执行转换 ==========
    # 使用xacro命令将XACRO文件转换为URDF文件
    print_info "开始转换..."
    if xacro "$xacro_file" -o "$urdf_file"; then
        # 转换成功
        print_success "转换成功: $urdf_file"
        
        # ========== 文件信息统计 ==========
        # 显示生成文件的详细信息
        if [ -f "$urdf_file" ]; then
            local file_size=$(du -h "$urdf_file" | cut -f1)      # 文件大小
            local line_count=$(wc -l < "$urdf_file")             # 行数
            local char_count=$(wc -c < "$urdf_file")             # 字符数
            
            print_info "文件统计信息:"
            echo "    📄 文件大小: $file_size"
            echo "    📝 行数: $line_count"
            echo "    🔤 字符数: $char_count"
            
            # 检查文件内容的基本有效性
            if grep -q "<robot" "$urdf_file" && grep -q "</robot>" "$urdf_file"; then
                print_success "URDF文件格式验证通过"
            else
                print_error "警告: URDF文件格式可能有问题"
            fi
        fi
        
        return 0
    else
        # 转换失败
        print_error "转换失败"
        print_info "可能的原因："
        echo "    1. XACRO文件语法错误"
        echo "    2. 缺少依赖的包或文件"
        echo "    3. ROS环境设置不正确"
        echo "    4. 权限问题"
        return 1
    fi
}

# ============================================================================== 
# 批量转换功能
# ==============================================================================

# 函数: convert_all_elfin
# 功能: 转换所有可用的Elfin机器人模型
convert_all_elfin() {
    print_info "开始批量转换所有Elfin机器人模型..."
    
    local success_count=0    # 成功转换的数量
    local total_count=0      # 总尝试转换的数量
    local failed_models=()   # 失败的模型列表
    
    # 遍历所有Elfin XACRO文件
    for xacro_file in "${URDF_DIR}"/*.urdf.xacro; do
        if [ -f "$xacro_file" ]; then
            total_count=$((total_count + 1))
            model_name=$(basename "$xacro_file" .urdf.xacro)
            
            echo
            print_info "处理模型 $total_count: elfin:$model_name"
            echo "----------------------------------------"
            
            # 转换单个模型
            if convert_model "elfin:$model_name"; then
                success_count=$((success_count + 1))
                print_success "✓ elfin:$model_name 转换成功"
            else
                failed_models+=("elfin:$model_name")
                print_error "✗ elfin:$model_name 转换失败"
            fi
        fi
    done
    
    # ========== 批量转换结果汇总 ==========
    echo
    echo "=========================================="
    print_info "批量转换完成 - 结果汇总"
    echo "=========================================="
    echo "📊 总计: $total_count 个模型"
    echo "✅ 成功: $success_count 个模型"
    echo "❌ 失败: $((total_count - success_count)) 个模型"
    
    # 显示成功转换的模型
    if [ $success_count -gt 0 ]; then
        echo
        print_success "成功转换的模型:"
        for xacro_file in "${URDF_DIR}"/*.urdf.xacro; do
            if [ -f "$xacro_file" ]; then
                model_name=$(basename "$xacro_file" .urdf.xacro)
                urdf_file="${URDF_DIR}/${model_name}.urdf"
                if [ -f "$urdf_file" ]; then
                    echo "    ✓ elfin:$model_name → $urdf_file"
                fi
            fi
        done
    fi
    
    # 显示失败的模型
    if [ ${#failed_models[@]} -gt 0 ]; then
        echo
        print_error "转换失败的模型:"
        for failed_model in "${failed_models[@]}"; do
            echo "    ✗ $failed_model"
        done
    fi
    
    echo
    if [ $success_count -eq $total_count ]; then
        print_success "🎉 所有模型转换成功！"
        return 0
    else
        print_error "⚠️  部分模型转换失败，请检查上述错误信息"
        return 1
    fi
}

# ============================================================================== 
# 主函数：程序入口点
# ==============================================================================

# 函数: main
# 参数: $1 - 可选的模型名称
# 功能: 主程序逻辑，处理用户输入并调用相应功能
main() {
    # ========== 程序标题 ==========
    echo "🤖 XACRO转URDF测试工具"
    echo "=" | head -c 50 | tr '\n' '='
    echo
    
    local model_name="$1"
    
    # ========== 交互式模型选择 ==========
    # 如果用户没有指定模型，显示可用模型并提示选择
    if [ -z "$model_name" ]; then
        list_models
        echo
        echo "💡 提示："
        echo "    - 输入完整格式: elfin:elfin3 或 ur:ur5"
        echo "    - 输入简短格式: elfin3 (默认为elfin类型)"
        echo "    - 输入 'all-elfin' 转换所有Elfin模型"
        echo
        read -p "请输入要转换的模型名称: " model_name
    fi
    
    # ========== 输入验证 ==========
    if [ -z "$model_name" ]; then
        print_error "未指定模型名称"
        print_info "使用 $0 --help 查看帮助信息"
        exit 1
    fi
    
    # ========== 特殊命令处理 ==========
    # 检查是否是批量转换命令
    if [ "$model_name" = "all-elfin" ]; then
        convert_all_elfin
        exit $?
    fi
    
    # ========== 单个模型转换 ==========
    print_info "转换模型: $model_name"
    echo
    
    # 调用转换函数
    if convert_model "$model_name"; then
        # ========== 转换成功后的操作指导 ==========
        echo
        print_success "🎉 转换完成！"
        echo
        print_info "后续操作建议："
        
        # 根据机器人类型提供相应的操作建议
        read -r model_type model_simple <<< "$(echo "$model_name" | sed 's/:/ /')"
        if [ -z "$model_simple" ]; then
            model_type="elfin"
            model_simple="$model_name"
        fi
        
        echo "📁 查看生成的URDF文件:"
        if [ "$model_type" = "ur" ]; then
            echo "    cat ${UR_URDF_DIR}/${model_simple}.urdf"
        else
            echo "    cat ${URDF_DIR}/${model_simple}.urdf"
        fi
        
        echo
        echo "🚀 启动RViz可视化:"
        if [ "$model_type" = "ur" ]; then
            echo "    export ROS_PACKAGE_PATH=\$PWD:\$ROS_PACKAGE_PATH"
            echo "    roslaunch ur_description view_${model_simple}.launch"
        else
            echo "    ./run_rviz.sh $model_name"
        fi
        
        echo
        echo "🎮 启动关节控制:"
        echo "    ./control_joints.py"
        echo "    # 或者"
        echo "    ./quick_joint_control.sh"
        
    else
        # ========== 转换失败处理 ==========
        print_error "转换失败"
        echo
        print_info "故障排除建议："
        echo "    1. 检查模型名称是否正确: $0 --list"
        echo "    2. 确保ROS环境已设置: source /opt/ros/\$ROS_DISTRO/setup.bash"
        echo "    3. 安装必要依赖: sudo apt install ros-\$ROS_DISTRO-xacro"
        echo "    4. 检查文件权限和磁盘空间"
        exit 1
    fi
}

# ============================================================================== 
# 命令行参数处理
# ==============================================================================

# 显示帮助信息
if [ "$1" = "-h" ] || [ "$1" = "--help" ]; then
    echo "XACRO转URDF转换工具 - 使用说明"
    echo "=========================================="
    echo
    echo "用法: $0 [选项] [model_name]"
    echo
    echo "选项:"
    echo "  -h, --help     显示此帮助信息"
    echo "  -l, --list     列出所有可用的机器人模型"
    echo
    echo "模型名称格式:"
    echo "  elfin:elfin3   # Elfin3机器人"
    echo "  ur:ur5         # UR5机器人"
    echo "  elfin3         # 简写格式（默认elfin类型）"
    echo "  all-elfin      # 转换所有Elfin模型"
    echo
    echo "可用的模型:"
    list_models | grep "✅" | awk '{print "  " $2}'
    echo
    echo "使用示例:"
    echo "  $0                    # 交互式选择模型"
    echo "  $0 elfin3           # 转换elfin3模型"
    echo "  $0 elfin:elfin5     # 转换elfin5模型（完整格式）"
    echo "  $0 ur:ur3           # 转换UR3机器人"
    echo "  $0 all-elfin        # 批量转换所有Elfin模型"
    echo "  $0 --list           # 列出所有可用模型"
    echo
    echo "注意事项:"
    echo "  - 确保已设置ROS环境变量"
    echo "  - 需要安装xacro工具包"
    echo "  - 生成的URDF文件将保存在对应的urdf目录中"
    exit 0
fi

# 列出所有可用模型
if [ "$1" = "-l" ] || [ "$1" = "--list" ]; then
    list_models
    exit 0
fi

# ========== 程序入口 ==========
# 调用主函数，传递第一个命令行参数
main "$1" 