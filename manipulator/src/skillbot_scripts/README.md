# SKILLBOT scripts

## Dankmux
 
Набор скриптов для управления tmux сессией.

`dankmux.py` создает сессию с необходимым количеством окон.
```
usage: dankmux [-h] [-n NAME] [-m] [-d] [shape1] [shape2]

programm for creating or manage tmux sessinon in weird way

positional arguments:
  shape1                horizontal amount of new windows or summary amount of new windows if [shape2] not specified
  shape2                vertical amount of new windows

optional arguments:
  -h, --help            show this help message and exit
  -n NAME, --name NAME  set tmux session name
  -m, --modify          modify mode to change existing session
```

`ldcfg.py` загружает конфиг терминалов
```
usage: dankmuxcfg [-h] [-d] [cfg]

programm for creating or manage tmux sessinon in weird way

positional arguments:
  cfg         path to cfg, by default search file in /config/, default value 'default.json'

optional arguments:
  -h, --help  show this help message and exit
  -d          start session detached
```

Конфиг представляет из себя json-файл. `default.json`:
```json
{
    "default_session_1" :
    [
        {"cmd" : "clear; echo hello1"},
        {"cmd" : "clear; echo hello2"},
        {"cmd" : "clear; echo hello3"},
        {"cmd" : "clear; echo hello3"},
        {"cmd" : "clear; echo hello3"},
        {"cmd" : "clear; echo hello4"}
    ]
}
```

`TODO:` Добавить разбиение по окнам.


## Topic mirror

Утилита дублирования топиков под другим названием.
```
usage: topic mirroring [-h] topic_from topic_to

positional arguments:
  topic_from
  topic_to

optional arguments:
  -h, --help  show this help message and exit
```
