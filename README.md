## Запуск контейнера

```console
docker build -t <image_name> --output=type=docker .
docker compose up -d
docker exec -it <container_name> bash
docker exec -it mas_high_level bash


```

## Сборка

```console
./build.py
```

### Сборка под Raspberry

```console
./build.py --rpi
```

### Пересборка с нуля

```console
./build.py --clean
```

## Деплой (можно и вне контейнера, если есть пакеты python3, sshpass и scp)

```console !!!!!!!!!!!!!!!!!НЕ РАБОТАЕТ!!!!!!!!!!!!!!!
./deploy.py <ip> <password>
```
или
```console
./deploy.py <user>@<ip> <password>
```
Если настроить правильно ssh ключи, можно будет убрать пароль из скрипта

## Перед запуском
