# tb3




## Getting started
1. Restore tb3 project in your ROS2 workspace (ex ws)
2. Build using colcon:
    - cd /home/ws/
    - colcon build --packages-select tb3_tele
3. Re-load environment variables:
    source /home/ws/install/setup.bash
4. Run:
    - teleop: ros2 run tb3_tele remote
    - image: ros2 run tb3_tele capture

## Add your files

- [ ] [Create](https://docs.gitlab.com/ee/user/project/repository/web_editor.html#create-a-file) or [upload](https://docs.gitlab.com/ee/user/project/repository/web_editor.html#upload-a-file) files
- [ ] [Add files using the command line](https://docs.gitlab.com/ee/gitlab-basics/add-file.html#add-a-file-using-the-command-line) or push an existing Git repository with the following command:

```
cd existing_repo
git remote add origin https://gitlab.upt.ro/ps1-2024/samples/tb3.git
git branch -M main
git push -uf origin main
```

