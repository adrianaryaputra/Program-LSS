# 🚢 Program LSS - Autonomous Surface Vehicle (ASV) Control System

[![Status Proyek](https://img.shields.io/badge/status-eksperimental-orange)](https://shields.io/)

Selamat datang di repositori Program LSS! Proyek ini adalah workspace ROS (Robot Operating System) yang didedikasikan untuk pengembangan dan kontrol kendaraan permukaan otonom (ASV).

**Proyek ini bersifat eksperimental dan dikelola oleh [Laboratorium Sistem dan Sibernetika](https://www.its.ac.id/see/id/penelitian/laboratorium/sistem-kendali-dan-sibernetika/), Departemen Teknik Elektro, Institut Teknologi Sepuluh Nopember (ITS), Surabaya.**

---

## 🌟 Fitur Utama

*   **Navigasi Otonom**: Kemampuan untuk mengikuti waypoint dan menjalankan misi secara mandiri.
*   **Kontrol Manual**: Opsi untuk mengendalikan ASV secara manual melalui GCS.
*   **Ground Control Station (GCS)**: Antarmuka pengguna berbasis Rviz untuk visualisasi, telemetri, dan interaksi.
*   **Modularitas**: Dibangun dengan paket-paket ROS yang terpisah untuk otonomi, kontrol, lokalisasi, dan GCS.
*   **Simulasi**: (Kemungkinan) Dukungan untuk simulasi sebelum penerapan di lapangan.

---

## 🏗️ Struktur Proyek

Repositori ini mengikuti struktur workspace Catkin standar. Berikut adalah paket-paket ROS utama dalam direktori `src/`:

*   `lss_autonomy`: Mengelola logika otonomi tingkat tinggi, perencanaan misi, dan navigasi.
*   `lss_controller`: Bertanggung jawab atas kontrol tingkat rendah ASV, seperti perintah ke motor.
*   `lss_localization`: Menangani estimasi posisi dan orientasi ASV.
*   `lss_gcs/`: Berisi paket-paket terkait Ground Control Station:
    *   `map_image`: Menyediakan dan memproses gambar peta.
    *   `nala_gcs_bringup`: File peluncuran untuk memulai sistem GCS dan komponen lainnya.
    *   `rviz_plugin`: Plugin Rviz khusus untuk visualisasi dan interaksi tingkat lanjut.
*   `lss_srvs`: Mendefinisikan tipe layanan ROS kustom untuk komunikasi antar node.

Untuk pemahaman teknis yang lebih mendalam bagi pengembang, silakan merujuk ke file `AGENTS.md`.

---

## 🛠️ Prasyarat

Sebelum memulai, pastikan sistem Anda memenuhi prasyarat berikut:

*   **ROS Noetic Ninjemys**: Distribusi ROS ini direkomendasikan.
    *   Instalasi: [ROS Noetic Installation Guide](http://wiki.ros.org/noetic/Installation/Ubuntu)
*   **Catkin Command Line Tools**: `python3-catkin-tools` atau `python3-catkin-pkg`.
*   **Git**: Untuk mengkloning repositori.

---

## ⚙️ Petunjuk Instalasi

Berikut adalah panduan untuk menginstal Program LSS pada berbagai platform:

### 🐧 Linux (Ubuntu 20.04 Focal Fossa Direkomendasikan)

1.  **Instal ROS Noetic**: Ikuti panduan resmi [ROS Noetic Installation Guide](http://wiki.ros.org/noetic/Installation/Ubuntu). Pastikan Anda menginstal versi `ros-noetic-desktop-full`.

2.  **Inisialisasi Catkin Workspace**:
    ```bash
    mkdir -p ~/lss_ws/src
    cd ~/lss_ws/
    catkin init  # Jika menggunakan catkin_tools
    # atau catkin_make jika menggunakan catkin_make tradisional
    ```

3.  **Kloning Repositori**:
    ```bash
    cd ~/lss_ws/src
    git clone <URL_REPOSITORI_ANDA> .
    # Ganti <URL_REPOSITORI_ANDA> dengan URL SSH atau HTTPS repositori ini
    ```

4.  **Instal Dependensi ROS**:
    Gunakan `rosdep` untuk menginstal dependensi paket dari direktori root workspace Anda:
    ```bash
    cd ~/lss_ws/
    rosdep install --from-paths src --ignore-src -r -y
    ```
    Jika `rosdep` belum diinisialisasi:
    ```bash
    sudo rosdep init
    rosdep update
    ```

5.  **Bangun Workspace**:
    ```bash
    cd ~/lss_ws/
    catkin build # Direkomendasikan jika menggunakan catkin_tools
    # atau
    # catkin_make
    ```

6.  **Source Lingkungan**:
    Tambahkan baris berikut ke file `.bashrc` atau `.zshrc` Anda agar tidak perlu sourcing setiap membuka terminal baru:
    ```bash
    echo "source ~/lss_ws/devel/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    # atau untuk zsh:
    # echo "source ~/lss_ws/devel/setup.zsh" >> ~/.zshrc
    # source ~/.zshrc
    ```
    Atau source secara manual di setiap terminal:
    ```bash
    source ~/lss_ws/devel/setup.bash
    ```

### 🖥️ Windows (Melalui Windows Subsystem for Linux - WSL2)

Cara termudah untuk menjalankan proyek ROS di Windows adalah dengan menggunakan WSL2.

1.  **Instal WSL2**: Ikuti panduan resmi Microsoft: [Install WSL](https://docs.microsoft.com/en-us/windows/wsl/install).
2.  **Instal Distribusi Linux (Ubuntu 20.04)**: Dari Microsoft Store, instal Ubuntu 20.04 LTS.
3.  **Konfigurasi Ubuntu di WSL2**: Setelah instalasi, buka Ubuntu 20.04 dari Start Menu.
4.  **Ikuti Petunjuk Instalasi Linux**: Setelah berada di dalam lingkungan Ubuntu WSL2, ikuti semua langkah yang dijelaskan di bagian "Linux (Ubuntu 20.04)" di atas.
5.  **X Server (Untuk GUI seperti Rviz)**: Untuk menjalankan aplikasi GUI ROS dari WSL2 di desktop Windows Anda, Anda memerlukan X Server untuk Windows (misalnya, VcXsrv, X410). Konfigurasikan X Server dan atur variabel `DISPLAY` di WSL2:
    ```bash
    echo "export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0" >> ~/.bashrc
    echo "export LIBGL_ALWAYS_INDIRECT=1" >> ~/.bashrc # Mungkin diperlukan
    source ~/.bashrc
    ```

### 🐳 Docker (Direkomendasikan untuk Portabilitas)

Menggunakan Docker adalah cara yang sangat baik untuk memastikan lingkungan yang konsisten dan portabel.

1.  **Instal Docker Engine**: Ikuti panduan resmi untuk sistem operasi Anda: [Install Docker Engine](https://docs.docker.com/engine/install/).
2.  **Dockerfile**: Sebuah `Dockerfile` akan (atau seharusnya) tersedia di root repositori ini. Jika belum ada, Anda mungkin perlu membuatnya berdasarkan prasyarat proyek (ROS Noetic, dependensi, dll.).
3.  **Bangun Docker Image**:
    Dari direktori root repositori (yang berisi `Dockerfile`):
    ```bash
    docker build -t lss_program .
    ```
4.  **Jalankan Docker Container**:
    Untuk menjalankan container dan mendapatkan shell interaktif:
    ```bash
    docker run -it --rm \
        --net=host \ # Untuk kemudahan akses jaringan ROS
        --env="DISPLAY" \ # Untuk GUI Forwarding
        --env="QT_X11_NO_MITSHM=1" \ # Untuk GUI Forwarding
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \ # Untuk GUI Forwarding
        # Tambahkan volume mount untuk kode Anda jika ingin mengedit secara live
        # --volume="${PWD}:/root/lss_ws/src/lss_program" \
        lss_program bash
    ```
    **Catatan untuk GUI (Rviz) di Docker**:
    *   Anda perlu memastikan X server Anda di host mengizinkan koneksi dari container. Jalankan `xhost +local:docker` di terminal host Anda sebelum menjalankan container.
    *   Perintah `docker run` di atas sudah menyertakan beberapa variabel lingkungan dan volume mount yang umum diperlukan untuk X11 forwarding.

---

## 🚀 Cara Menjalankan

1.  **Pastikan ROS Master Berjalan**:
    Di terminal pertama, jalankan `roscore`:
    ```bash
    roscore
    ```

2.  **Luncurkan Sistem Utama**:
    Di terminal kedua (setelah sourcing `devel/setup.bash` atau di dalam container Docker yang sudah disiapkan):
    ```bash
    roslaunch nala_gcs_bringup all.launch
    ```
    Ini seharusnya akan memulai semua node yang diperlukan, termasuk GCS (Rviz).

3.  **Menjalankan Misi Spesifik (Contoh)**:
    Skrip misi seperti `mission_control_ILOS.py` atau `mission_control_PLOS.py` dapat dijalankan jika node-node yang relevan dari sistem utama sudah aktif.
    Di terminal lain:
    ```bash
    python <NAMA_FILE_MISI>.py
    # Contoh: python mission_control_ILOS.py
    ```

---

## 🤝 Kontribusi

Kontribusi untuk pengembangan Program LSS sangat diharapkan! Jika Anda menemukan bug, memiliki ide untuk fitur baru, atau ingin meningkatkan dokumentasi, silakan:

1.  Buka **Issue** untuk mendiskusikan perubahan atau melaporkan masalah.
2.  Buat **Pull Request** dengan perubahan yang Anda usulkan.

---

## 📜 Lisensi

Proyek ini dilisensikan di bawah **Lisensi MIT**. Lihat file `LICENSE` untuk detail lebih lanjut.

---

## 🙏 Ucapan Terima Kasih

Proyek ini merupakan bagian dari upaya penelitian dan pengembangan di **Laboratorium Sistem dan Sibernetika, Departemen Teknik Elektro, Institut Teknologi Sepuluh Nopember (ITS), Surabaya**. Terima kasih kepada semua pihak yang telah berkontribusi.
