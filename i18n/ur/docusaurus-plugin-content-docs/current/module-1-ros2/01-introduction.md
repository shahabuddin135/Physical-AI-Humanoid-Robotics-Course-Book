---
description: ROS 2 کا تعارف - وہ middleware جو جدید روبوٹس کو طاقت دیتا ہے۔ سیکھیں
  کہ یہ کیوں موجود ہے، یہ کیوں اہم ہے، اور آپ اسے کیوں پسند بھی کریں گے اور اس سے
  بیزار بھی ہوں گے۔
id: ros2-introduction
keywords:
- ROS 2
- Robot Operating System
- middleware
- robotics
- rclpy
- nodes
module: 1
sidebar_position: 1
tags:
- module-1
- fundamentals
- beginner
title: ROS 2 کیوں؟ (ایک محبت-نفرت کی کہانی)
---

# ROS 2 کیوں؟ (ایک محبت اور نفرت کی کہانی)

> **مختصر خلاصہ:** ROS 2 وہ middleware ہے جو روبوٹ کے مختلف حصوں کو ایک دوسرے سے بات چیت کرنے دیتا ہے۔ یہ دراصل ایک آپریٹنگ سسٹم نہیں ہے (نام جھوٹ ہے)، لیکن یہ روبوٹکس میں ایک عالمی معیار کے قریب ترین چیز ہے۔

---

## روبوٹ سافٹ ویئر کے بارے میں سچی حقیقت

یہ ایک گندا راز ہے جو روبوٹکس انڈسٹری نئے سیکھنے والوں کو نہیں بتاتی: **شروع سے ایک روبوٹ بنانا ایک ڈراؤنا خواب ہے**۔

ہارڈ ویئر کی وجہ سے نہیں—وہ صرف مہنگا اور مایوس کن ہوتا ہے۔ اصل ڈراؤنا خواب سافٹ ویئر ہے۔ تصور کریں کہ آپ کے پاس ہے:

- ایک **camera** جو 30fps پر تصاویر آؤٹ پٹ کرتا ہے
- ایک **LiDAR** جو گھومتا ہے اور ہر سیکنڈ 100,000 پوائنٹس خارج کرتا ہے
- چھ **motors** جنہیں ہر 10 ملی سیکنڈ میں مربوط کمانڈز کی ضرورت ہوتی ہے
- ایک **path planning algorithm** جسے حساب لگانے میں 500ms لگتے ہیں
- ایک **AI model** جو ایک GPU پر چل رہا ہے اور... کچھ ذہین کام کر رہا ہے

اب ان سب کو ایک ساتھ کام کروائیں۔ real-time میں۔ کریش ہوئے بغیر۔ ایک ایسے روبوٹ پر جو حرکت کر رہا ہو۔

*یہ وہ جگہ ہے جہاں لوگ پینا شروع کر دیتے ہیں۔*

---

## ROS کا تعارف (ابتدائی گناہ)

2007 میں، Willow Garage کے کچھ ذہین/پاگل انجینئرز نے کہا: "کیا ہو اگر ہم روبوٹ کے اجزاء کے لیے بات چیت کا ایک معیاری طریقہ بنائیں؟"

انہوں نے **ROS (Robot Operating System)** بنایا۔ نام کے باوجود، یہ ایک آپریٹنگ سسٹم نہیں ہے—یہ middleware ہے۔ یہ tools، libraries، اور conventions کا ایک سیٹ ہے جو روبوٹ کے مختلف حصوں کو ایک دوسرے سے بات چیت کرنے دیتا ہے۔

ROS 1 انقلابی تھا۔ یہ بھی تھا:

- **real-time نہیں تھا** (آپ کا روبوٹ بازو شاید سست ہو کر... دیوار سے ٹکرا جائے)
- **صرف ایک machine کے لیے** (distributed systems کے ساتھ اچھی قسمت)
- ایک "Master" node پر **منحصر** (ایک کریش = سب کچھ ختم)
- **اس وقت لکھا گیا جب Python 2 مقبول تھا** (2020 نے کال کی، یہ مر چکا ہے)

لیکن یہ کام کرتا تھا! کسی حد تک۔ زیادہ تر وقت۔ اور روبوٹکس کمیونٹی نے اسے اپنا لیا کیونکہ متبادل یہ تھا کہ ہر کوئی پہیہ دوبارہ ایجاد کرے (لفظی طور پر—motor controllers)۔

---

## ROS 2: الیکٹرک بوگالو

2017 میں، ROS 2 لیبز سے ابھرا، جسے ROS 1 پر ایک دہائی کی لعنت ملامت سے سیکھے گئے اسباق کے ساتھ شروع سے دوبارہ لکھا گیا۔

### کیا بدلا؟

| فیچر | ROS 1 | ROS 2 |
|---------|-------|-------|
| **Real-time support** | نہیں 😅 | ہاں! (کچھ شرائط کے ساتھ) |
| **Multi-machine** | مشکل | Native |
| **Central master** | ضروری | ختم! Peer-to-peer |
| **Python version** | 2.7 💀 | 3.x 🎉 |
| **Security** | LOL | اصل encryption |
| **Industry backing** | تعلیمی | Amazon, Intel, NVIDIA, وغیرہ |

### فن تعمیر (آسان کردہ)

```
┌─────────────────────────────────────────────────────────┐
│                     YOUR ROBOT                          │
│                                                          │
│  ┌──────────┐    ┌──────────┐    ┌──────────┐          │
│  │  Camera   │    │  LiDAR   │    │   Motor   │          │
│  │   Node    │    │   Node   │    │   Node    │          │
│  └─────┬────┘    └─────┬────┘    └─────┬────┘          │
│        │              │              │                 │
│        └──────────────┼──────────────┘                 │
│                       │                                 │
│              ┌────────▼────────┐                       │
│              │  DDS Middleware │  ← The magic layer     │
│              │ (Discovery/Pub- │                        │
│              │  Sub/Services)  │                        │
│              └────────┬────────┘                       │
│                       │                                 │
│        ┌──────────────┼──────────────┐                 │
│        │              │              │                 │
│  ┌─────▼────┐   ┌─────▼────┐   ┌─────▼────┐          │
│  │Navigation│   │  AI/ML   │   │ Planning │           │
│  │   Node   │   │   Node   │   │   Node   │           │
│  └──────────┘   └──────────┘   └──────────┘           │
│                                                          │
└─────────────────────────────────────────────────────────┘
```

---

## آپ کو کیوں پرواہ کرنی چاہیے

اگر آپ کام کرنا چاہتے ہیں:

- **Boston Dynamics** کے روبوٹس پر → وہ ROS 2 استعمال کرتے ہیں
- **Amazon warehouse robots** پر → ROS 2
- **NASA کے lunar rovers** پر → اندازہ لگائیں... ROS 2
- **آپ کے اپنے startup کے humanoid** پر → آپ ROS 2 استعمال کریں گے

یہ سنجیدہ روبوٹکس کے لیے **de facto standard** ہے۔ اسے پسند کریں یا ناپسند کریں، آپ کو اسے جاننا ضروری ہے۔

---

## اس ماڈیول میں ہم کیا کور کریں گے

اس ماڈیول کے اختتام تک، آپ سمجھ جائیں گے:

1.  **Nodes** — بنیادی building blocks (جیسے microservices، لیکن روبوٹس کے لیے)
2.  **Topics** — Nodes پیغامات کیسے نشر کرتے ہیں (pub/sub pattern)
3.  **Services** — Request/response communication (جب آپ کو جواب کی ضرورت ہو)
4.  **rclpy** — Python client library (کیونکہ C++ کے لیے زندگی بہت مختصر ہے)
5.  **URDF** — اپنے روبوٹ کے جسم کو XML میں کیسے بیان کریں (ہاں، XML، 2024 میں)

### وقت کی پابندی

⏱️ **تخمینہ شدہ وقت:** 3-4 گھنٹے کا مرکوز کام

### پیشگی ضروریات

- Python 3.8+ (آپ جانتے ہیں کہ چیزوں کو `pip install` کیسے کرنا ہے)
- pub/sub patterns کی بنیادی سمجھ (اختیاری، ہم اسے کور کریں گے)
- Linux کو ترجیح دی جاتی ہے (WSL2 کام کرتا ہے، macOS ایک جوا ہے، native Windows... مہم جویانہ ہے)
- کبھی کبھار پراسرار error messages کے لیے صبر

---

## سیٹ اپ (آئیے اسے ختم کریں)

اس سے پہلے کہ ہم کوڈ میں غوطہ لگائیں، آپ کو ROS 2 انسٹال کرنے کی ضرورت ہوگی۔ موجودہ LTS ورژن **Humble Hawksbill** ہے (ہاں، نام سمندری کچھوے ہیں، پوچھیں نہیں)۔

### Ubuntu 22.04 (تجویز کردہ)

```bash
# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Add ROS 2 apt repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble
sudo apt update
sudo apt install ros-humble-desktop

# Add to your .bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### انسٹالیشن کی تصدیق کریں

```bash
ros2 --version
# Should output: ros2 0.x.x (Humble)
```

اگر آپ کو ورژن نمبر نظر آتا ہے، مبارک ہو! آپ روبوٹ بنانے کے لیے تیار ہیں۔ اگر نہیں... تو بہت سے debugging سیشنز میں سے پہلے میں خوش آمدید۔

---

## اگلا

اگلے حصے میں، ہم اپنے پہلے ROS 2 nodes بنائیں گے اور سیکھیں گے کہ وہ کیسے بات چیت کرتے ہیں۔ روبوٹس کو ایک دوسرے سے بات کرنے (اور کبھی کبھار بحث کرنے) کے لیے تیار ہو جائیں۔

**→ [Nodes, Topics, and Services](/docs/module-1-ros2/02-nodes-topics-services)**