# lidar_mapping:
# edited by lichunjing 2021.02.28:

### 1. 分支跟踪情况

本地分支                                   远程分支                        远程服务器别名
master: 全部代码分支         --->           github: master                 origin
master-gitlab: 稳定版分支    --->           gitlab: master                 origin-gitlab

常用命令：
1. 将本地的 master 分支同步到 github 远程的 master 分支
```
git push origin master:master
```

2. 将本地的 master-git 分支同步到 gitlab 远程的 master 分支
```
git push origin-gitlab master-gitlab:master
```


