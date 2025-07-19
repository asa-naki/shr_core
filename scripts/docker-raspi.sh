#!/bin/bash
# Raspberry Pi用Docker環境の構築・実行スクリプト

set -e

# 設定
IMAGE_NAME="ros2-raspi-dev"
TAG="jazzy"
CONTAINER_NAME="ros2-raspi-dev"
COMPOSE_FILE="docker/docker-compose.raspi.yml"

# カラー出力
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

print_status() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# ヘルプ表示
show_help() {
    cat << EOF
Raspberry Pi ROS2 Docker Management Script

使用方法:
  $0 [COMMAND] [OPTIONS]

コマンド:
  build         Dockerイメージをビルド
  run           コンテナを起動
  exec          実行中のコンテナに接続
  stop          コンテナを停止
  logs          コンテナのログを表示
  clean         コンテナとイメージを削除
  status        コンテナの状態を表示
  setup         初回セットアップ
  help          このヘルプを表示

オプション:
  --no-cache    キャッシュを使わずにビルド
  --force       強制実行
  --follow      ログを継続表示

例:
  $0 setup                 # 初回セットアップ
  $0 build                 # イメージビルド
  $0 run                   # コンテナ起動
  $0 exec                  # コンテナに接続
  $0 logs --follow         # ログを継続表示
EOF
}

# アーキテクチャ確認
check_architecture() {
    local arch=$(uname -m)
    if [[ "$arch" != "aarch64" ]]; then
        print_warning "このスクリプトはARM64/aarch64アーキテクチャ用です"
        print_warning "現在のアーキテクチャ: $arch"
        print_warning "クロスプラットフォームビルドを試行します..."
        export DOCKER_BUILDKIT=1
    fi
}

# Docker環境チェック
check_docker() {
    if ! command -v docker &> /dev/null; then
        print_error "Dockerがインストールされていません"
        exit 1
    fi
    
    if ! command -v docker-compose &> /dev/null; then
        print_error "Docker Composeがインストールされていません"
        exit 1
    fi
    
    if ! docker info &> /dev/null; then
        print_error "Dockerデーモンが実行されていません"
        exit 1
    fi
}

# 必要なディレクトリを作成
setup_directories() {
    print_status "必要なディレクトリを作成中..."
    mkdir -p workspace
    mkdir -p logs
    mkdir -p config
    print_success "ディレクトリ作成完了"
}

# X11フォワーディングの設定
setup_x11() {
    if [[ -n "$DISPLAY" ]]; then
        print_status "X11フォワーディングを設定中..."
        xhost +local:docker 2>/dev/null || print_warning "xhostコマンドが利用できません"
        export DISPLAY=${DISPLAY}
    fi
}

# Dockerイメージのビルド
build_image() {
    local no_cache=""
    if [[ "$1" == "--no-cache" ]]; then
        no_cache="--no-cache"
    fi
    
    print_status "Raspberry Pi用ROS2 Dockerイメージをビルド中..."
    docker build $no_cache \
        -f docker/Dockerfile.raspi \
        -t ${IMAGE_NAME}:${TAG} \
        --platform linux/arm64 \
        .
    
    print_success "イメージビルド完了: ${IMAGE_NAME}:${TAG}"
}

# コンテナの起動
run_container() {
    print_status "Raspberry Pi ROS2コンテナを起動中..."
    
    # 既存のコンテナを停止
    if docker ps -q -f name=${CONTAINER_NAME} &> /dev/null; then
        print_warning "既存のコンテナを停止中..."
        docker stop ${CONTAINER_NAME} &> /dev/null || true
    fi
    
    # X11設定
    setup_x11
    
    # docker-composeで起動
    docker-compose -f ${COMPOSE_FILE} up -d ros2-raspi-dev
    
    print_success "コンテナが起動しました"
    print_status "コンテナに接続するには: $0 exec"
}

# コンテナに接続
exec_container() {
    if ! docker ps -q -f name=${CONTAINER_NAME} &> /dev/null; then
        print_error "コンテナが実行されていません"
        print_status "起動するには: $0 run"
        exit 1
    fi
    
    print_status "コンテナに接続中..."
    docker exec -it ${CONTAINER_NAME} bash
}

# コンテナの停止
stop_container() {
    print_status "コンテナを停止中..."
    docker-compose -f ${COMPOSE_FILE} down
    print_success "コンテナを停止しました"
}

# ログ表示
show_logs() {
    local follow=""
    if [[ "$1" == "--follow" ]]; then
        follow="-f"
    fi
    
    docker logs $follow ${CONTAINER_NAME}
}

# 状態表示
show_status() {
    print_status "Docker環境の状態:"
    echo ""
    echo "=== イメージ ==="
    docker images ${IMAGE_NAME} || echo "イメージが見つかりません"
    echo ""
    echo "=== コンテナ ==="
    docker ps -a --filter name=${CONTAINER_NAME} || echo "コンテナが見つかりません"
    echo ""
    echo "=== ボリューム ==="
    docker volume ls --filter name=ros2 || echo "ボリュームが見つかりません"
}

# クリーンアップ
clean_all() {
    print_status "Docker環境をクリーンアップ中..."
    
    # コンテナ停止・削除
    docker-compose -f ${COMPOSE_FILE} down -v 2>/dev/null || true
    
    # イメージ削除
    docker rmi ${IMAGE_NAME}:${TAG} 2>/dev/null || true
    
    # ボリューム削除
    docker volume prune -f
    
    print_success "クリーンアップ完了"
}

# 初回セットアップ
setup_environment() {
    print_status "Raspberry Pi ROS2 Docker環境の初回セットアップを開始..."
    
    check_docker
    check_architecture
    setup_directories
    
    print_status "Dockerイメージをビルドしています..."
    build_image
    
    print_success "セットアップ完了!"
    print_status "次のステップ:"
    print_status "1. コンテナを起動: $0 run"
    print_status "2. コンテナに接続: $0 exec"
}

# メイン処理
main() {
    case "${1:-help}" in
        build)
            check_docker
            check_architecture
            build_image $2
            ;;
        run)
            check_docker
            setup_directories
            run_container
            ;;
        exec)
            exec_container
            ;;
        stop)
            stop_container
            ;;
        logs)
            show_logs $2
            ;;
        status)
            show_status
            ;;
        clean)
            clean_all
            ;;
        setup)
            setup_environment
            ;;
        help|--help|-h)
            show_help
            ;;
        *)
            print_error "不明なコマンド: $1"
            show_help
            exit 1
            ;;
    esac
}

# スクリプト実行
main "$@"
