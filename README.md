## ROS 2 Humble Docker Projesi

Bu proje, Docker içinde çalışan üç düğümlü bir ROS 2 Humble uygulamasıdır:

- **sensor_publisher_pkg**: Sahte sensör verisi üretip `/sensor_value` topiğine yayınlar.
- **data_processor_pkg**: `/sensor_value` verisini dinler, işler ve sonucu `/processed_value` topiğine yayınlar.
- **command_server_pkg**: `/compute_command` servisini sunar ve gelen değere göre `"HIGH"` veya `"LOW"` çıktısı üretir.

### Düğüm Özeti

- **sensor_publisher**
  - Topic: `/sensor_value` (`std_msgs/msg/Float32`)
  - Periyot: 0.1 s
  - Varsayılan veri: `random.uniform(0, 20)` (isterseniz kodu düzenleyerek sinüs veya sayaç yapabilirsiniz)

- **data_processor**
  - Subscriber: `/sensor_value` (`std_msgs/msg/Float32`)
  - Publisher: `/processed_value` (`std_msgs/msg/Float32`)
  - İşleme: Değer 2 ile çarpılır: `processed = data * 2`

- **command_server**
  - Service: `/compute_command`
  - Özel servis türü: `command_server_interfaces/srv/ComputeCommand`
    - `float64 input`
    - `---`
    - `string output`
  - Mantık:
    - `input > 10` → `"HIGH"`
    - Aksi halde → `"LOW"`

### Proje Klasör Yapısı

```text
project_root /
  Dockerfile
  entrypoint.sh
  launch /
    my_project.launch.py
  src /
    sensor_publisher_pkg /
    data_processor_pkg /
    command_server_pkg /
    command_server_interfaces /
    my_project /
  SSF_HASH.txt
  README.md
  ssf.sh
```

### Docker ile Çalıştırma

1. İmajı derle:

```bash
git clone https://github.com/Selami7321/env3_Robotik.git
cd env3_Robotik
sudo docker build -t myrosapp .
```

2. Konteyneri çalıştır:

```bash
sudo docker run --rm --name rosapp myrosapp
```

3. Ayrı bir terminalde çalışan konteynere gir:

```bash
sudo docker ps                  # container_id'yi not edin
sudo docker exec -it <container_id> bash
```

4. Konteyner içinde ROS ortamını yükleyin ve test komutlarını çalıştırın:

```bash
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
ros2 topic list
ros2 topic echo /processed_value
ros2 service call /compute_command command_server_interfaces/srv/ComputeCommand "{input: 12.5}"
```

### Notlar

- Tüm doğrulamalar Docker konteyneri içinde yapılır; host üzerinde ROS 2 kurulumu gerekli değildir.
- `sensor_publisher` düğümünde sensör veri üretim türünü (rastgele, sinüs, sayaç) değiştirmek için ilgili Python dosyasındaki kodu düzenleyebilirsiniz.

### Gereksinimler

- Docker **20.10+**
- Git **2.30+**
- Host makinede ROS 2 kurulumu **gerekmiyor** (her şey Docker içinde çalışıyor).

### Projeyi Klonlama ve Çalıştırma (Başka Kullanıcılar İçin)

1. Depoyu klonlayın:

```bash
git clone https://github.com/Selami7321/env3_Robotik.git
cd env3_Robotik
```

2. Docker imajını oluşturun:

```bash
sudo docker build -t myrosapp .
```

3. Konteyneri çalıştırın:

```bash
sudo docker run --rm --name rosapp myrosapp
```

4. Ayrı bir terminalde çalışan konteynere girin:

```bash
sudo docker ps                  # container_id'yi not edin
sudo docker exec -it <container_id> bash
```

5. Konteyner içinde ROS ortamını yükleyin ve doğrulama komutlarını çalıştırın:

```bash
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
ros2 topic list
ros2 topic echo /processed_value
ros2 service call /compute_command command_server_interfaces/srv/ComputeCommand "{input: 12.5}"
```

### Bu Projeyi Kendi GitHub Hesabınıza Taşımak (Terminal ile)

1. Bu projeyi klonlayın:

```bash
git clone https://github.com/Selami7321/env3_Robotik.git
cd <orijinal-repo>
```

2. Mevcut `origin` remote'unu kaldırın:

```bash
git remote remove origin
```

3. GitHub hesabınızda **boş** bir repo oluşturun (örnek: `myrosapp-ros2-humble`).

4. Yeni remote'u ekleyin:

```bash
git remote add origin https://github.com/<sizin-kullanici-adiniz>/myrosapp-ros2-humble.git
```

5. Kodları kendi reponuza gönderin:

```bash
git branch -M main
git push -u origin main
```


