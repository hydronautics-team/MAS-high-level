У подписчика и обработчика топика planner_message сейчас неправильный тип сообщения, строка была для тестов
planner_msg_callback - обработчик сообщений в топик planner_message, тут надо распарсить как при приеме, но наоборот
Следи за портами. В твоем случае сервер только принимает, а клиент отправляет. Питоновские скрипты только для тестов
Вся подготовка сообщения происходит в обработчике прерываний. Это плохо. Есть шанс, что когда частота сообщений дойдет до придельной, что может либо упасть, либо будут теряться пакеты. По хорошему надо делать буффер сообщений в виде очередей и таймеры. Это можно сделать как в самой ноде, так и на стороне udp сервера и клиента. Или таймеры на стороне ноды, а очереди на стороне сервера и клиента. Тут дело вкуса.