alias i:= install
alias bi:= build-image
alias b:= build
alias r:= run
alias e:= enter

container_name:='forg_bot'
docker-compose-service:='forg_bot_app'

default:
  just --list

install:
  @echo "Not implemented"

build-image args="--progress='auto'": down
  @echo "Use 'just build-image --progress=\"plain\"' for more information. Options: auto (default), tty, plain, json, quiet"
  cd ros2_docker && docker compose {{args}} -f docker-compose.local.yml build;
  docker compose {{args}} -f docker-compose.local.yml build;


build: down && down
  @echo "Not implemented"
  cd ../.. && colcon build --symlink-install
#   docker compose -f docker-compose.local.yml run --rm --name {{container_name}} {{docker-compose-service}} bash ./scripts/build.sh

CHANGED_FILES := `git diff --name-only origin/develop...HEAD | tr '\n' ' '`

# lint files=CHANGED_FILES: down && down
#   echo "Only linting these files: {{files}}"
#   docker compose -f docker-compose.local.yml run --rm --name {{container_name}} {{docker-compose-service}} bash -c "./scripts/lint-cpp.sh {{files}}"

run: down && down
  docker compose -f docker-compose.local.yml run -it --rm --name {{container_name}} {{docker-compose-service}} zsh

enter:
  docker exec -it {{container_name}} zsh

down:
  docker stop {{container_name}} || true
  docker rm {{container_name}} || true
  docker compose -f docker-compose.local.yml down || true

unit-test: down && down
  # docker compose -f docker-compose.local.yml run --rm {{docker-compose-service}} bash ./test/cpputest/build_and_test.sh
