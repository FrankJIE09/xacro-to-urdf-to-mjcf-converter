### 项目名称：**GitGuard**

---

### 项目简介
**GitGuard** 是一个 Git 项目初始化工具集，旨在帮助开发者快速配置 Git 仓库的最佳实践，包括：
- 自动初始化 Git 仓库。
- 配置 `.gitignore` 文件，避免提交常见的临时文件和依赖文件。
- 设置 `pre-commit` 钩子，阻止超过 50MB 的文件提交，确保代码库清洁。
- 提供简单易用的测试功能，验证钩子配置是否有效。

通过 **GitGuard**，开发者可以减少手动配置的繁琐步骤，快速开始高效协作开发。

---

### 文件说明

| **文件名称**          | **功能描述**                                                                                   |
|-----------------------|-----------------------------------------------------------------------------------------------|
| `setup_pre_commit.sh` | 配置 Git 的 `pre-commit` 钩子，阻止提交超过 50MB 的文件。                                      |
| `create_gitignore.sh` | 创建或更新 `.gitignore` 文件，添加常见的忽略规则，如日志文件、编译输出文件、临时文件等。        |
| `setup_git_project.sh`| 集成初始化脚本，包括 Git 初始化、配置 `.gitignore` 文件、设置 `pre-commit` 钩子的一站式工具。 |

---

### 使用方法

#### 1. 克隆项目到本地
```bash
git clone https://github.com/yourusername/gitguard.git
cd gitguard
```

#### 2. 为脚本赋予执行权限
```bash
chmod +x setup_pre_commit.sh create_gitignore.sh setup_git_project.sh
```

#### 3. 脚本使用
##### **单独配置 `pre-commit` 钩子**
运行以下命令，仅设置文件大小检查钩子：
```bash
./setup_pre_commit.sh
```

##### **单独配置 `.gitignore` 文件**
运行以下命令，仅创建或更新 `.gitignore` 文件：
```bash
./create_gitignore.sh
```

##### **完整项目初始化**
运行以下命令，完成 Git 初始化、`.gitignore` 配置和 `pre-commit` 钩子设置：
```bash
./setup_git_project.sh
```

---

### 测试功能
运行 `setup_pre_commit.sh` 或 `setup_git_project.sh` 时，系统会提示是否进行测试：
1. 创建一个 60MB 的测试文件。
2. 验证提交是否被阻止。
3. 自动清理测试文件。

---

### 常见问题
1. **Git 未初始化或路径错误**
   - 确保当前目录是 Git 项目目录。
   - 如果不是，请运行 `setup_git_project.sh` 自动初始化。

2. **权限问题**
   - 脚本需要可执行权限，请运行以下命令：
     ```bash
     chmod +x setup_pre_commit.sh create_gitignore.sh setup_git_project.sh
     ```

3. **钩子未生效**
   - 确保 Git 版本在 2.9 以上，并正确配置了 `.git/hooks` 目录的文件权限。

---

### 项目特点
- **简单高效**：一步到位完成常见配置，无需手动编辑。
- **安全保证**：阻止大文件提交，避免代码库膨胀。
- **灵活扩展**：可以根据项目需求，轻松修改 `.gitignore` 和 `pre-commit` 配置。

---

### 贡献
欢迎提交 Issue 或 Pull Request 来改进 **GitGuard**！

---

### 许可证
**GitGuard** 遵循 MIT 许可证，详情请查看 `LICENSE` 文件。