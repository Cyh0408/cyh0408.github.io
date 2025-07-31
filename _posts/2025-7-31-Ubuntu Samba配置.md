
# Ubuntu Samba配置
---

## 1. Ubuntu服务器配置

### 1.1 安装samba

```bash
sudo apt install samba samba-common
```

### 1.2 创建共享目录并分配权限

```bash
sudo mkdit -p /home/$USER/share
sudo chmod 777 /home/$USER/share
```

### 1.3 配置samba

```bash
sudo gedit /etc/samba/smb.conf
```

在文件末尾添加共享配置：

```bash
[share]
comment = Ubuntu Shared Folder
path = /home/$USER/share
browseable = yes
guest ok = yes
read only = no
create mask = 0777
directory mask = 0777
```

### 1.4 设置samba用户（可选）

```bash
sudo smbpasswd -a $USER
```

### 1.5 启动samba服务

```bash
sudo systemctl start smbd
```

### 1.6 设置开机自启动

```bash
sudo systemctl enable smbd
```

## 2. Windows访问共享文件夹

### 2.1 方法一：文件夹地址栏输入ubuntu的IP地址进行访问

```bash
// 192.168.xxx.xxx
```

### 2.2 方法二：打开cmd输入上述命令进行访问