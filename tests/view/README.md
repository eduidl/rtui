## Info Panel

```sh-session
# Node
poetry run python -m tests.view.test_info_panel --node <node_name>

# Topic
poetry run python -m tests.view.test_info_panel --topic <topic_name>

# Service
poetry run python -m tests.view.test_info_panel --service <node_name>

# Action
poetry run python -m tests.view.test_info_panel --action <topic_name>
```

## List Panel

```sh-session
# Node
poetry run python -m tests.view.test_list_panel --type [node, topic, service, action]
```
