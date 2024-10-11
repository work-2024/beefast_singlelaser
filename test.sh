!/bin/bash

echo "运行测试代码..."
echo "1:运行fox发布环境..."
echo "2:设置生命周期状态..."
echo "3:运行代码..."
echo "4:退出代码..."

source ~/.bashrc
source ./install/setup.bash

# 读取用户输入  
read -p "请输入选项（1-4）: " choice
# 根据用户输入执行相应的操作  
case $choice in  
    1)  
        ros2 launch foxglove_bridge foxglove_bridge_launch.xml address:=172.16.20.184 &> /dev/null &
        ;;

    2)  
        ros2 lifecycle set /lifecycle_cliff_detection  configure
        wait
        ros2 lifecycle set /lifecycle_cliff_detection  activate
        wait
        ros2 lifecycle set /lifecycle_single_laser configure
        wait
        ros2 lifecycle set /lifecycle_single_laser activate
        wait
        ;;  
    3)  
        ros2 launch beefast_single_laser ssl20n.launch.py
        wait
        ;;  
    4)  
        echo "退出程序。"  
        exit 0  
        ;;  
    *)  
        echo "无效输入，请输入1到3之间的数字。"  
        # 可以选择在这里重新显示菜单或退出  
        ;;  
esac
exit 0 