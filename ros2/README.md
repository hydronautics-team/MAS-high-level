## Запуск контейнера

docker build -t <image_name> --output=type=docker .
docker compose up -d
docker exec -it <container_name> bash

## Сборка

./build.py

### Сборка под Raspberry

./build.py --rpi

### Пересборка с нуля

./build.py --clean

## Деплой (можно и вне контейнера, если есть пакеты python3, sshpass и scp)

./deploy.py <ip> <password>
или
./deploy.py <user>@<ip> <password>