# ログを消すとコードが軽くなることがある — arbiter の実例

- ログは文字列を出すだけでなく、**そのログのためだけに API・型・テストへ漏れ出した依存**を連れていることがある
- その依存ごと畳めるなら、ログ削除は立派なリファクタになる
- `autoware_traffic_light_arbiter` での実例を 2 つ、diff リンクで示す（リンク先が変更の全体）

## どんなときに効くか

- ログに渡している値に、**ログ以外の消費者がいない**
- なのに、その値を運ぶためだけの戻り値 / out-param / struct / テストが生えている
- → それは機能ではなく「観測のための税」。ログを消せば、まとめて畳める

## 実例 1: `RCLCPP_DEBUG` 1 行を消す → [diff](https://github.com/takam5f2/autoware_universe/commit/efa2a2535923731d1c31801ea6a9f54b54e5bb62)

- 一番効く例。本番では既定 off の DEBUG なので **振る舞いは一切変わらない**
- この 1 行だけが、「ログされるためだけに存在するデータ経路」の唯一の利用者だった
- 消えたもの: `id`/`age` を運ぶ `struct ExpiredExternalSignal`、`ingest_perception` の戻り値、`struct ExternalIngestResult`、Node のログ用ヘルパ
- 結果: `ingest_perception` は `void`、`ingest_external` は `bool` を返すだけの素直な API に
- その eviction の中身を検証するだけのテストも 1 本不要に（sweep の実挙動は別テストが継続カバー）
- 5 ファイル / +32 −91 行

## 実例 2: `RCLCPP_WARN_THROTTLE` を全廃 → [diff](https://github.com/takam5f2/autoware_universe/commit/02149205a715a065ad43b1f2d3ba619bb816e4aa)

- 4 本のうち 2 本は、Core が返す **診断データ専用** だった
- 消えたもの: `ArbitrationResult` の `off_map_signal_ids` と `latest_input_time`、`route_signal` の out-param、それらを pin するテスト群
- 結果: `ArbitrationResult` は `output` 1 個だけの素直な struct に
- 4 ファイル / +15 −100 行
- 注意: 残り 2 本は純粋な運用ログ（データを運んでいない）。消すと「黙ってドロップ」になるので、これは可視性の別判断。"全廃" を見せるために消しただけで、ログ削除が常に無料という意味ではない

## 逆に、安易に消さないログ

- ドロップ・タイムアウト・設定ミスを知らせる運用ログは、型を太らせていなくても運用価値がある → 可視性の観点で判断
- 例: 設定ミス時の WARN は「消す」より「fail-fast」が筋（silent fallback の別論点）

## レビュー時のチェックリスト

- そのログの引数、ログ以外に誰か使っている?
- その引数のためだけに型・戻り値・テストが増えていない?
- ログを消したら、それらは消える? → 消えるなら畳める候補

## ブランチ

- `demo/autoware_traffic_light_arbiter/remove-logs-simplifies-api`（fork `takam5f2` に push 済み）
- diff: [実例 1（DEBUG）](https://github.com/takam5f2/autoware_universe/commit/efa2a2535923731d1c31801ea6a9f54b54e5bb62) / [実例 2（WARN_THROTTLE）](https://github.com/takam5f2/autoware_universe/commit/02149205a715a065ad43b1f2d3ba619bb816e4aa)
- ビルド済み・Core テスト 36/36 PASS で検証済み
