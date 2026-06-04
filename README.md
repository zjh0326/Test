已自动排除 MDK-ARM 编译输出、Unity Library/Temp/Logs/obj 等临时目录。后续更新只需：

cd D:\motor-control
# 重新从工作目录复制最新文件
robocopy "D:\Keil_v5\Projects\6.CAN" "stm32" /E /XD "MDK-ARM"
robocopy "C:\Unity\UnityProjects\MotorControl" "unity" /E /XD "Library" "Logs" "Temp" "obj" ".vs"
# 提交推送
git add -A && git commit -m "update" && git push
