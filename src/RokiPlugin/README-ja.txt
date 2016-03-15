**Rokiをインストールする。**

 http://www.mi.ams.eng.osaka-u.ac.jp/open-j.htmlからダウンロードできます。
 
 最初にCureをダウンロードして、展開します。
 READMEに従ってインストールします。
このとき、インストール先のファルダーは予め作成し、その中にbinとlibのファルダーも作成しておいたほうが良いようです。
環境変数PATHとLD_LIBRARY_PATHにこのディレクトリを追記しておきます。

Zm Zeo Rokiの順に同じようにインストールします。


**CCmakeの設定**

BUILD_ROKI_PLUGINをONにして、ROKI_DIRにインストール先のフォルダーを指定します。


** 関節ダイナミクスのシミュレーションをするには**

choreonoidで、このシミュレーションの設定を行うためには、yamlファイルを使用します。
シミュレーション対象モデルを記述したｗｒｌファイルの他にyamlファイルを作成し、以下のように記述し、
yamlファイルをchoreonoidで読み込みます。

modelFile: arm_2dof.wrl　　　　　ｗｒｌファイルを指定します。

RokiJointParameters:　　　　　　　　　　　　　Roki用のパラメータであることを指定します。
  - name: Joint1　　　　　　　　　　　　　　　　　関節名を指定し、以下にその関節パラメータを記述します。
    motorconstant: 2.58e-2　　　　　　　　　モータ定数（トルク定数）
    admitance: 0.42373　　　　　　　　　　　　　端子間アドミッタンス（端子間抵抗の逆数）
    minvoltage: -24.0　　　　　　　　　　　　　　最小入力電圧
    maxvoltage: 24.0　　　　　　　　　　　　　　　最大入力電圧
    inertia: 1.65e-6　　　　　　　　　　　　　　　モータ回転子慣性モーメント
    gearinertia: 5.38e-6　　　　　　　　　　　減速機慣性モーメント
    ratio: 120.0　　　　　　　　　　　　　　　　　　　減速比
    stiff: 0.0000000000　　　　　　　　　　　　関節剛性係数
    viscos: 2.2　　　　　　　　　　　　　　　　　　　　関節粘性係数
    coulomb: 4.32　　　　　　　　　　　　　　　　　　関節乾性係数（動摩擦トルク）
    staticfriction: 4.92　　　　　　　　　　　最大静止摩擦トルク
  - name: Joint2　　　　　　　　　　　　　　　　　関節毎に繰り返し設定します。
    motorconstant: 2.58e-2
    admitance: 0.42373
　　　　...............

RokiArm2Dof.cnoidがこのサンプルです。


**破壊のシミュレーションをするには**

破断の起こる箇所を関節としてｗｒｌファイルに記述します。
関節のタイプはfreeとします。
そして、関節ダイナミクスの場合と同様にyamlファイルを使用し、以下のように記述します。

modelFile: breakWall.wrl　　　　　ｗｒｌファイルを指定します。

RokiJointParameters:　　　　　　　　　　　　　Roki用のパラメータであることを指定します。
  - name: link1　　　　　　　　　　　　　　　　　　関節名を指定し、以下にその破断パラメータを記述します。
    break: [ 200.0, 200.0 ]　　　　　　　　破断が起こる力・トルクのノルム閾値　（力、トルクの順）　
  - name: link2　　　　　　　　　　　　　　　　　　関節毎に繰り返し設定します。
    break: [ 10.0, 10.0 ]

choreonoidにモデルを読み込み、モデルアイテムの自己干渉検出のプロパティをtrueにします。
（trueにしないと、破断した後の物体が、お互いにすり抜けてしまいます。）
RokiSimulationaアイテムの全リンク位置姿勢出力のプロパティをtrueにします。

RokiBreakWall.cnoidがこのサンプルです。





