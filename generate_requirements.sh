#!/bin/bash

# 设置根目录为当前目录
ROOT_DIR="."
REQUIREMENTS_FILE="./requirements.txt"

# 检查 pipreqs 是否安装
if ! command -v pipreqs &> /dev/null; then
    echo "📦 pipreqs 未安装，正在安装..."
    pip install pipreqs
else
    echo "📦 pipreqs 已安装"
fi

echo "🔍 正在扫描项目目录以生成依赖列表..."

# 强制生成 requirements.txt
pipreqs "$ROOT_DIR" --force --encoding=utf-8 --savepath "$REQUIREMENTS_FILE"

# 检查 pipreqs 命令是否成功执行
if [ $? -eq 0 ]; then
    echo "✅ 依赖列表已初步生成到 $REQUIREMENTS_FILE"

    # --- 新增部分：获取并添加系统和Python信息 ---

    # 获取 Python 版本 (尝试 python3, 如果失败则尝试 python)
    if command -v python3 &> /dev/null; then
        PY_VERSION=$(python3 --version 2>&1)
    else
        PY_VERSION=$(python --version 2>&1)
    fi

    # 获取操作系统信息
    OS_INFO=$(uname -a)

    # 创建包含系统信息的临时文件
    TMP_FILE=$(mktemp)

    # 将头部信息写入临时文件
    {
        echo "# --- Environment Information ---"
        echo "# Generated on: $(date)"
        echo "# Operating System: $OS_INFO"
        echo "# Python Version: $PY_VERSION"
        echo "# -----------------------------"
        echo "" # 添加一个空行以增加可读性
    } > "$TMP_FILE"

    # 将原始 requirements.txt 的内容追加到临时文件
    cat "$REQUIREMENTS_FILE" >> "$TMP_FILE"

    # 用新生成的文件替换原始文件
    mv "$TMP_FILE" "$REQUIREMENTS_FILE"

    echo "✍️  已将 Python 版本和系统信息写入 $REQUIREMENTS_FILE 的顶部。"
    echo "🎉 全部完成！"
else
    echo "❌ 生成 requirements.txt 失败，请检查项目结构或依赖"
fi