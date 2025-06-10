#!/bin/bash

# ==============================================================================
# XACRO转URDF并启动RViz可视化脚本 (简化版)
#
# 功能: 
# 1. 将指定的xacro文件转换为urdf文件
# 2. 启动RViz显示机器人模型
# 3. 提供关节状态控制界面
#
# 用法: ./run_rviz.sh [model_name]
# 示例: ./run_rviz.sh elfin3
#       ./run_rviz.sh elfin5
#
# 如果不提供模型名称，脚本会列出可用模型供选择
# ==============================================================================

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 工作目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ELFIN_DESCRIPTION_DIR="${SCRIPT_DIR}/elfin_description"
UR_DESCRIPTION_DIR="${SCRIPT_DIR}/ur_description"
URDF_DIR="${ELFIN_DESCRIPTION_DIR}/urdf"
UR_URDF_DIR="${UR_DESCRIPTION_DIR}/urdf"
LAUNCH_DIR="${ELFIN_DESCRIPTION_DIR}/launch"

# 支持的机器人模型
AVAILABLE_ELFIN_MODELS=("elfin3" "elfin5" "elfin10" "elfin15" "elfin5_l" "elfin10_l")
AVAILABLE_UR_MODELS=("ur3" "ur5" "ur10" "ur3e" "ur5e" "ur10e" "ur16e")

# 打印函数
print_info() {
    echo -e "${BLUE}ℹ️  $1${NC}"
}

print_success() {
    echo -e "${GREEN}✅ $1${NC}"
}

print_error() {
    echo -e "${RED}❌ $1${NC}"
}

print_warning() {
    echo -e "${YELLOW}⚠️  $1${NC}"
}

print_header() {
    echo -e "${BLUE}"
    echo "🤖 XACRO转URDF并启动RViz可视化工具"
    echo "=" | head -c 50 | tr '\n' '='
    echo -e "${NC}"
}

# 检查依赖
check_dependencies() {
    print_info "检查依赖..."

    # 检查ROS环境
    if [ -z "$ROS_DISTRO" ]; then
        print_error "未找到ROS环境变量"
        echo "请先source您的ROS环境，例如:"
        echo "  source /opt/ros/noetic/setup.bash"
        exit 1
    fi

    # 检查xacro命令
    if ! command -v xacro &> /dev/null; then
        print_error "xacro命令未找到"
        echo "请安装xacro: sudo apt install ros-$ROS_DISTRO-xacro"
        exit 1
    fi

    # 检查roslaunch
    if ! command -v roslaunch &> /dev/null; then
        print_error "roslaunch命令未找到"
        echo "请安装ROS完整版或检查ROS安装"
        exit 1
    fi

    print_success "依赖检查通过"
}

# 检查文件结构
check_file_structure() {
    print_info "检查文件结构..."

    if [ ! -d "$ELFIN_DESCRIPTION_DIR" ]; then
        print_error "elfin_description目录不存在: $ELFIN_DESCRIPTION_DIR"
        exit 1
    fi

    if [ ! -d "$URDF_DIR" ]; then
        print_error "urdf目录不存在: $URDF_DIR"
        exit 1
    fi

    if [ ! -f "$ELFIN_DESCRIPTION_DIR/urdf.rviz" ]; then
        print_error "rviz配置文件不存在: $ELFIN_DESCRIPTION_DIR/urdf.rviz"
        exit 1
    fi

    # 检查UR description目录（可选）
    if [ -d "$UR_DESCRIPTION_DIR" ]; then
        print_success "检测到UR机器人支持"
    fi

    print_success "文件结构检查通过"
}

# 列出可用模型
list_available_models() {
    print_info "可用的机器人模型:"
    local available=()

    # 检查Elfin机器人
    echo "  📁 Elfin 机器人:"
    for model in "${AVAILABLE_ELFIN_MODELS[@]}"; do
        if [ -f "$URDF_DIR/${model}.urdf.xacro" ]; then
            echo "    ✅ elfin:$model"
            available+=("elfin:$model")
        fi
    done

    # 检查UR机器人
    if [ -d "$UR_DESCRIPTION_DIR" ]; then
        echo "  📁 UR (Universal Robot) 机器人:"
        for model in "${AVAILABLE_UR_MODELS[@]}"; do
            # 检查URDF文件是否存在（可能是.urdf或从launch生成）
            if [ -f "$UR_URDF_DIR/${model}.urdf" ] || [ -f "$SCRIPT_DIR/${model}.urdf" ]; then
                echo "    ✅ ur:$model"
                available+=("ur:$model")
            fi
        done
    fi

    echo "${available[@]}"
}

# 交互式模型选择
select_model_interactive() {
    local available_models_str
    available_models_str=$(list_available_models)

    # 提取可用模型到数组
    local available=()

    # 添加Elfin机器人
    for model in "${AVAILABLE_ELFIN_MODELS[@]}"; do
        if [ -f "$URDF_DIR/${model}.urdf.xacro" ]; then
            available+=("elfin:$model")
        fi
    done

    # 添加UR机器人
    if [ -d "$UR_DESCRIPTION_DIR" ]; then
        for model in "${AVAILABLE_UR_MODELS[@]}"; do
            if [ -f "$UR_URDF_DIR/${model}.urdf" ] || [ -f "$SCRIPT_DIR/${model}.urdf" ]; then
                available+=("ur:$model")
            fi
        done
    fi

    if [ ${#available[@]} -eq 0 ]; then
        print_error "没有找到可用的机器人模型"
        exit 1
    fi

    echo
    echo "请选择一个机器人模型:"
    for i in "${!available[@]}"; do
        echo "  $((i+1)). ${available[$i]}"
    done

    while true; do
        read -p "请输入选择 (1-${#available[@]}): " choice
        if [[ "$choice" =~ ^[0-9]+$ ]] && [ "$choice" -ge 1 ] && [ "$choice" -le ${#available[@]} ]; then
            echo "${available[$((choice-1))]}"
            return
        else
            print_error "无效选择，请重新输入"
        fi
    done
}

# 解析机器人类型和型号
parse_robot_type() {
    local full_model="$1"

    if [[ "$full_model" == *":"* ]]; then
        # 格式: robot_type:model_name
        echo "${full_model%%:*}" "${full_model##*:}"
    else
        # 兼容旧格式，默认为elfin
        echo "elfin" "$full_model"
    fi
}

# 转换xacro到urdf
convert_xacro_to_urdf() {
    local full_model="$1"
    local robot_type model_name
    read -r robot_type model_name <<< "$(parse_robot_type "$full_model")"

    case "$robot_type" in
        "elfin")
            local xacro_file="$URDF_DIR/${model_name}.urdf.xacro"
            local urdf_file="$URDF_DIR/${model_name}.urdf"

            if [ ! -f "$xacro_file" ]; then
                print_error "xacro文件不存在: $xacro_file"
                exit 1
            fi

            print_info "转换 ${model_name}.urdf.xacro 到 ${model_name}.urdf..."

            # 设置ROS包路径
            export ROS_PACKAGE_PATH="$SCRIPT_DIR:${ROS_PACKAGE_PATH:-}"

            # 使用xacro命令转换
            if xacro "$xacro_file" -o "$urdf_file"; then
                print_success "转换成功: $urdf_file"
            else
                print_error "转换失败"
                exit 1
            fi
            ;;
        "ur")
            # UR机器人的URDF已经生成，或者使用launch文件动态生成
            local urdf_file="$SCRIPT_DIR/${model_name}.urdf"
            if [ ! -f "$urdf_file" ]; then
                urdf_file="$UR_URDF_DIR/${model_name}.urdf"
            fi

            if [ -f "$urdf_file" ]; then
                print_success "找到UR机器人URDF文件: $urdf_file"
            else
                print_info "UR机器人将通过launch文件动态生成URDF"
            fi
            ;;
        *)
            print_error "不支持的机器人类型: $robot_type"
            exit 1
            ;;
    esac
}

# 启动roscore（如果需要）
start_roscore_if_needed() {
    print_info "检查roscore状态..."

    # 检查roscore是否已经运行
    if timeout 2 rostopic list &> /dev/null; then
        print_success "roscore已在运行"
        return
    fi

    print_info "启动roscore..."
    # 在后台启动roscore
    nohup roscore &> /dev/null &

    # 等待roscore启动
    local count=0
    while ! timeout 1 rostopic list &> /dev/null; do
        sleep 1
        count=$((count + 1))
        if [ $count -gt 10 ]; then
            print_error "roscore启动超时"
            exit 1
        fi
    done

    print_success "roscore已启动"
}



# 启动rviz显示
launch_rviz() {
    local full_model="$1"
    local robot_type model_name
    read -r robot_type model_name <<< "$(parse_robot_type "$full_model")"
    
    print_info "启动RViz显示 $robot_type:$model_name 模型..."
    
    # 设置ROS包路径
    export ROS_PACKAGE_PATH="$SCRIPT_DIR:$ROS_PACKAGE_PATH"
    
    case "$robot_type" in
        "elfin")
            # 启动elfin display.launch
            print_info "执行命令: roslaunch elfin_description display.launch model:=$model_name"
            
            echo
            print_success "RViz即将启动"
            echo "📝 使用说明:"
            echo "  - 在RViz中您可以看到Elfin${model_name}机器人模型"
            echo "  - 使用joint_state_publisher GUI调节关节角度"
            echo "  - 按 Ctrl+C 退出程序"
            echo
            
            # 启动roslaunch
            roslaunch elfin_description display.launch model:="$model_name"
            ;;
        "ur")
            # 启动UR view launch
            print_info "执行命令: roslaunch ur_description view_${model_name}.launch"
            
            echo
            print_success "RViz即将启动"
            echo "📝 使用说明:"
            echo "  - 在RViz中您可以看到UR${model_name}机器人模型"
            echo "  - 使用joint_state_publisher GUI调节关节角度"
            echo "  - 按 Ctrl+C 退出程序"
            echo
            echo "⚠️  注意: 如果遇到PyQt5错误，请确保在mujoco_dazu环境中运行"
            echo
            
            # 启动roslaunch
            roslaunch ur_description view_"${model_name}".launch
            ;;
        *)
            print_error "不支持的机器人类型: $robot_type"
            exit 1
            ;;
    esac
}

# 清理函数
cleanup() {
    if [ "${CLEANUP_CALLED:-}" != "true" ]; then
        export CLEANUP_CALLED="true"
        print_info "正在清理..."
        # 杀死相关进程
        pkill -f "roslaunch.*elfin_description" 2>/dev/null || true
        pkill -f "roslaunch.*ur_description" 2>/dev/null || true
        pkill -f "rviz" 2>/dev/null || true
        pkill -f "joint_state_publisher" 2>/dev/null || true
        print_success "清理完成"
    fi
}

# 信号处理
trap cleanup EXIT INT TERM

# 主函数
main() {
    print_header
    
    # 检查依赖和文件结构
    check_dependencies
    check_file_structure
    
    local model_name="$1"
    
    # 如果未提供模型名称，交互式选择
    if [ -z "$model_name" ]; then
        model_name=$(select_model_interactive)
    else
        # 验证提供的模型名称
        read -r robot_type model_check <<< "$(parse_robot_type "$model_name")"
        
        case "$robot_type" in
            "elfin")
                if [ ! -f "$URDF_DIR/${model_check}.urdf.xacro" ]; then
                    print_error "模型 '$model_name' 不可用"
                    echo
                    list_available_models
                    exit 1
                fi
                ;;
            "ur")
                # 检查UR模型是否支持
                found=false
                for available_model in "${AVAILABLE_UR_MODELS[@]}"; do
                    if [ "$available_model" == "$model_check" ]; then
                        found=true
                        break
                    fi
                done
                if [ "$found" != true ]; then
                    print_error "模型 '$model_name' 不可用"
                    echo
                    list_available_models
                    exit 1
                fi
                ;;
            *)
                print_error "不支持的机器人类型: $robot_type"
                echo
                list_available_models
                exit 1
                ;;
        esac
    fi
    
    print_info "选择的模型: $model_name"
    echo
    
    # 转换xacro到urdf
    convert_xacro_to_urdf "$model_name"
    
    # 启动roscore（如果需要）
    start_roscore_if_needed
    
    # 启动rviz显示
    launch_rviz "$model_name"
}

# 帮助信息
show_help() {
    echo "用法: $0 [选项] [模型名称]"
    echo
    echo "选项:"
    echo "  -h, --help     显示此帮助信息"
    echo "  -l, --list     列出所有可用的机器人模型"
    echo
    echo "可用的机器人模型:"
    list_available_models | grep -E "✅" | awk '{print "  " $2}'
    echo
    echo "示例:"
    echo "  $0                    # 交互式选择模型"
    echo "  $0 elfin3           # 直接启动elfin3模型"
    echo "  $0 --list           # 列出所有可用模型"
}

# 参数处理
case "${1:-}" in
    -h|--help)
        show_help
        exit 0
        ;;
    -l|--list)
        list_available_models
        exit 0
        ;;
    -*)
        print_error "未知选项: $1"
        show_help
        exit 1
        ;;
    *)
        main "$1"
        ;;
esac 