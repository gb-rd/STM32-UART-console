Задание.
Разработать программу для stm32 со следующим функционалом:
1. Программа на микроконтроллере должна иметь UART интерфейс для соединения с компьютером через COM порт и программу для работы с консолью, например putty
2. Программа на микроконтроллере должна принимать текстовые команды, через UART интерфейс, от компьютера и отвечать на них так же в текстовом виде.
3. Программа на микроконтроллере должна выполнять следующие команды:
3.1 reset - Перезагружает контроллер, перед перезагрузкой контроллер должен сообщить подключённому устройству сообщение о том что контроллер будет перезагружен
3.2 led=[on|off] - Проецирует полученное значение команды на набортный светодиод. Для примера команда led=on включает светодиод, а команда led=off выключает. В ответ микроконтроллер должен сообщать о текущем состоянии светодиода.
3.3 status - Получение текущего состояния светодиода. В ответ контроллер должен сообщить текущее состояние светодиода в виде сообщения len=on или len=off
4. Программа должна принимать команды без учета регистра.
5. Программа должна иметь защиту от переполнения буфера.
