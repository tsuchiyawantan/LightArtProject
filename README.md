# An Interactive System for Light-Art-Like Representation of Human Silhouettes

Light art represents various objects by a light stroke drawn in the air. It takes about 10 to 30 seconds to create a light art picture, binding these pictures we could create a light art movie. It is a time-consuming task because we need large number of frames to create a movie. This code presents a light-art-like interactive system.  
ペンライトアートとは，ペンライトで空中に絵を描き，光の軌跡を記録するアートのことである.ペンライトアート写真を制作する時間は，約10秒から30秒掛かるため，ペンライトアート写真を繋ぎ合わせたペンライトアート動画を制作するためには，膨大な時間を必要とする.そこで，ペンライトアート風描画を実時間でインタラクティブに実現することで，インタラクティブアートとペンライトアートに着目した新たな表現手法を提案する. 本システムはKinectの人認識機能によって人物だけの身体のエッジを，深度データを用いて実時間で取得し，ライトアートのような表現をするアートシステムである.最終的には，ペンライトアートの特徴である，一筆書き描画と手書き風デフォルメを実装することで，ペンライトアート風での描画を目標とする.

## Description
This system extracts edges of human body by using depth information and then displays them as light art in real time. 

***DEMO:***

![Demo](koidance-afim.gif)

## Features
C++, Kinect, OpenCV, realtime, light-art-like, dance 
