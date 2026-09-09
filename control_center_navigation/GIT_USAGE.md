# Git 简要使用说明

本功能包已经建立 git 基线，用来在修改功能后快速查看差异、保存版本和回退。

## 进入功能包

```bash
cd ~/ros2_ws/src/control_center_navigation
```

## 查看当前改了什么

```bash
git status
git diff
```

## 保存一次可回退版本

修改确认可用后提交：

```bash
git add .
git commit -m "说明这次修改内容"
```

例子：

```bash
git commit -m "add waypoint visualization"
```

## 回退未提交的修改

如果只是改了文件，还没有提交，想恢复到上一次提交：

```bash
git restore .
```

误删文件也可以用同一条命令恢复：

```bash
git restore .
```

## 回退到当前基线

当前基线提交是：

```text
6fc4cca baseline: control_center_navigation
```

如果想强制回到这个基线：

```bash
git reset --hard 6fc4cca
```

注意：这会丢弃基线之后所有未保存的修改。

## 查看历史版本

```bash
git log --oneline
```

## 回到某个历史版本

先用 `git log --oneline` 找到提交号，然后执行：

```bash
git reset --hard 提交号
```

## 修改后重新编译

回到 `ros2_ws` 编译这个包：

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select control_center_navigation
source install/setup.bash
```

## 推荐习惯

每次大改前先确认工作区是干净的：

```bash
git status
```

每次功能验证通过后提交一次：

```bash
git add .
git commit -m "简短说明"
```
