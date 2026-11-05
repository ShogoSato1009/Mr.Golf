# 3Dゴルフゲーム開発規約

## 目的と作業範囲

C++で3Dゴルフゲームを開発する。
このアプリのソースコードと開発履歴は、GitHubリポジトリ [ShogoSato1009/Mr.Golf](https://github.com/ShogoSato1009/Mr.Golf) で管理する。
Wii Sports Resortのゴルフを参考に、ボールの物理挙動、操作性、グラフィックを段階的に実装する。
各セッションではユーザーが指定した範囲を実装し、将来の機能を先行実装しない。
具体的な実装方法や使用ライブラリは、以下の規約とその時点の要件に沿って判断する。

## 設計原則

- 機能ごとにファイルを分け、各モジュールの責務を明確にする。
- 物理演算、操作入力、描画を独立させ、修正の影響を局所化する。
- モジュール間は明示的なデータ構造や必要最小限のインターフェースで接続する。
- 依存関係は一方向とし、循環依存を禁止する。
- 過度な抽象化、不要な継承階層、現段階で不要な機能を導入しない。

## 基本構成と責務

以下はプロジェクトルートからの構成案であり、必要になる段階でファイルを作成する。
ヘッダーと実装の分離は必要に応じて行う。

```text
AGENTS.md
CMakeLists.txt
src/
  main.cpp
  core/
    Math.hpp
    Config.hpp
  physics/
    BallState.hpp
    BallPhysics.hpp / .cpp
    Collision.hpp / .cpp
  world/
    Terrain.hpp / .cpp
    Environment.hpp
  shot/
    ShotCommand.hpp
    ShotMapper.hpp / .cpp
  input/
    InputSystem.hpp / .cpp
    SwingAnalyzer.hpp / .cpp
  rendering/
    Renderer.hpp / .cpp
    Camera.hpp / .cpp
  demo/
    DebugConfig.hpp / .cpp
    Simulation.hpp / .cpp
    RunDemo.hpp / .cpp
  game/
    GameController.hpp / .cpp
tests/
configs/
tools/
```

| モジュール | 責務 |
| --- | --- |
| `core` | 共通の数学型・処理と設定。上位モジュールに依存しない。 |
| `physics` | ボール状態、運動、衝突、反発、転がりの計算。 |
| `world` | 地形と環境条件の提供。入力や描画に依存しない。 |
| `input` | 操作機器からの入力取得とスイング情報の解析。 |
| `shot` | 解析済みの操作情報を物理的なショット指令に変換。 |
| `rendering` | 状態の表示とカメラ制御。物理状態を変更しない。 |
| `demo` | 検証用ショットの設定、固定刻みの実行、描画時刻のサンプリング、プレビューと動画用フレームの出力。 |
| `game` | モジュールの接続、更新順序、ゲーム進行、UIの統合。 |
| `main.cpp` | 起動・終了と初期化。個別のゲーム処理を集約しない。 |

## データの流れと依存境界

```text
InputSystem → SwingAnalyzer → ShotMapper → ShotCommand → BallPhysics → BallState → Renderer
Terrain / Environment → BallPhysics
```

この図はデータの流れを示すものであり、クラス同士の直接依存を要求しない。
`GameController`が接続と進行を管理し、下位モジュールから参照されない構造とする。

- `ShotCommand`は入力方式に依存しないデータ構造とする。
  力積・角力積など、物理演算が受け取る指令を表す。
- `ShotMapper`は操作情報を`ShotCommand`へ変換する。
  入力機器への直接アクセスやボールの運動計算は行わない。
- `BallPhysics`は物理的な入力、地形、環境条件から運動を計算する。
  操作機器、`InputSystem`、`SwingAnalyzer`、`ShotMapper`、描画システムに依存しない。
  `ShotCommand`を参照する場合も、変換処理への依存を持ち込まない。
- `Terrain`と`Environment`は物理計算に必要な情報を提供する。
  地形の取得・問い合わせと、衝突応答の計算を分離する。
- `Renderer`は`BallState`などを読み取り専用で参照し、物理演算に介入しない。
- 第2段階の動作検証では`demo`が物理と描画を接続する。`demo`にゲーム進行や操作入力を持たせず、`physics`、`world`、`rendering`から`demo`を参照しない。
  デバッグ用の設定は`configs/`、フレームをMP4に変換するツールは`tools/`に置く。

## 実装順序

1. 基本データ構造と数学処理
2. ボールの物理演算
3. 衝突・反発・転がり
4. 地形と環境条件
5. 3D描画とカメラ
6. 操作入力とスイング解析
7. ショット変換
8. ゲームシステムとUI

物理演算を入力機器や描画なしで実行・検証できる状態にしてから、操作系を接続する。
先行段階で必要な地形・環境情報は、最小限のデータやインターフェースで提供する。

## 実装・変更時の確認事項

- 物理更新は固定時間刻みを基本とし、描画フレームレートから独立させる。
- 物理パラメータは設定や引数を通じて外部から変更可能にし、計算処理内へ埋め込まない。
- 各モジュールは可能な限り単体テスト可能にする。
  物理演算の検証に描画や実機入力を必須としない。
- 既存モジュールの変更時は、依存先と利用側への影響、循環依存の有無を確認する。
- 新機能は既存の責務分担を維持して追加する。
  ファイル構成や責務を変更する場合は、理由と依存関係への影響を説明し、この文書も更新する。
- 既存の設計・指示・ユーザーの変更を確認してから編集し、不用意に上書きしない。
