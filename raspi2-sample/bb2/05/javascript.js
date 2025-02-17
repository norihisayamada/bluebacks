// タッチのサポート状況のチェック用変数
var support = {
    pointer: window.navigator.pointerEnabled,
    mspointer: window.navigator.msPointerEnabled,
    touch: 'ontouchstart' in window
};

// タッチの場合わけ。pointer系：IE11以降、MSPointer系：IE10、touch系：android、iPhone、iPad
var touchStart = support.pointer ? 'pointerdown' :
                 support.mspointer ? 'MSPointerDown' : 'touchstart';
var touchMove =  support.pointer ? 'pointermove' :
                 support.mspointer ? 'MSPointerMove' : 'touchmove';
var touchEnd =   support.pointer ? 'pointerup' :
                 support.mspointer ? 'MSPointerUp' : 'touchend';

function initialize_webiopi(){
    // webiopiの準備が終わってからstyles.cssを適用する
    applyCustomCss('styles.css');

    // GPIOの状態を監視しない
    webiopi().refreshGPIO(false);

    mCanvas = document.getElementById("canvas");
    mCtx = mCanvas.getContext('2d');

    resize_canvas();

    // タッチエリアの設定
    var touchArea = $("#touchArea")[0];

    // タッチイベントのイベントリスナーの登録
    touchArea.addEventListener(touchStart, touchEvent, false);
    touchArea.addEventListener(touchMove, touchEvent, false);
    touchArea.addEventListener(touchEnd, touchEndEvent, false);

    // クリックイベントのイベントリスナーの登録
    touchArea.addEventListener("click", clickEvent, false);

    // Firefoxで画面の回転を検出
    var mqOrientation = window.matchMedia("(orientation: portrait)");
    mqOrientation.addListener(function() {
        resize_canvas();
    });

    // ウインドウサイズの変更を検出
    window.addEventListener('resize', function (event) {
        resize_canvas();
    });
}

// iOSで画面の回転を検出
window.onorientationchange = function()
{
    var iR = Math.abs( window.orientation );
    if ( iR == 0 || iR == 90 ){
        resize_canvas();
    }
}

var commandID = 0;
var prevStatus = "s";

var mTouchOffsetTop;
var mTouchOffsetLeft;

var mCount = 0;
var mCanvas;
var mCtx;
var mImg1;
var mImg2;
var mImgArrow;
var mWidth = 640;
var mHeight = 480;

var host = location.host;
var hostname = host.split(":")[0];
var port= 9000;
var URL1 = 'http://' + hostname + ':' + port + '/?action=snapshot';
var URL2 = 'http://' + hostname + ':8000/bb2/05/img/CrawlerControllerTrans.png';

var sliderMin = 0;
var sliderMax = 20;
var sliderStep = 1;
var sliderValue = sliderMax/2;

$(function() {
    var sliderHandler1 = function(e, ui){
        var ratio = ui.value/sliderMax;
        // サーボの回転の向きを逆にしたい場合次の行を無効に
        //ratio = 1.0 - ratio;
        webiopi().callMacro("setPCA9685PWM", [1, ratio, commandID++]);
    };
    var sliderHandler0 = function(e, ui){
        var ratio = ui.value/sliderMax;
        // サーボの回転の向きを逆にしたい場合次の行を無効に
        //ratio = 1.0 - ratio;
        webiopi().callMacro("setPCA9685PWM", [0, ratio, commandID++]);
    };

    $( "#slider1" ).slider({
        orientation: "vertical",
        min: sliderMin,
        max: sliderMax,
        step: sliderStep,
        value: sliderValue,
        change: sliderHandler1,
        slide: sliderHandler1
    });
    $( "#slider0" ).slider({
        orientation: "horizontal",
        min: sliderMin,
        max: sliderMax,
        step: sliderStep,
        value: sliderValue,
        change: sliderHandler0,
        slide: sliderHandler0
    });
});

function resize_canvas(){

    if($(window).width() < 4*$(window).height()/3){
        isPortrait = true;
    }else{
        isPortrait = false;
    }

    if(isPortrait){
        mWidth = 0.9*$(window).width();
        mHeight = 3*mWidth/4;
    }else{
        mHeight = 0.9*$(window).height();
        mWidth = 4*mHeight/3;
    }

    mCanvas.width = mWidth;
    mCanvas.height = mHeight;

    mTouchOffsetLeft = $("#canvas").offset().left;
    mTouchOffsetTop = $("#canvas").offset().top;

    $( "#slider1" ).height(mHeight);
    $( "#slider0" ).width(mWidth);

    imageSetup();
}

function imageSetup(){

    mImg1 = new Image();
    mImg2 = new Image();
    mImgArrow = new Image();

    mImg1.src = URL1 +'&'+(mCount++);

    mImgArrow.src = URL2;

    mImg1.onload = function() {
        mImg2.src = URL1 + '&' + (mCount++);
        mCtx.drawImage(mImg1, 0, 0, mWidth, mHeight);
        mCtx.drawImage(mImgArrow, 0, 0, mWidth, mHeight);
    };

    mImg2.onload = function() {
        mImg1.src =URL1 + '&' + (mCount++);
        mCtx.drawImage(mImg2, 0, 0, mWidth, mHeight);
        mCtx.drawImage(mImgArrow, 0, 0, mWidth, mHeight);
    };
}

function touchEvent(e){
    e.preventDefault();

    // タッチ中のイベントのみ捕捉(IE)
    if(support.pointer || support.mspointer){
        if(e.pointerType != 'touch' && e.pointerType != 2){
            return;
        }
    }
    var touch = (support.pointer || support.mspointer) ? e : e.touches[0];

    if(touch.pageX < mTouchOffsetLeft ||
       touch.pageX >= mTouchOffsetLeft + mWidth ||
       touch.pageY < mTouchOffsetTop ||
       touch.pageY >= mTouchOffsetTop + mHeight){
        return;
    }

    if(touch.pageX < mTouchOffsetLeft + mWidth/3){ // 左旋回

        if(prevStatus != "l"){
            webiopi().callMacro("set6LegsAction", ["l", commandID++]);
        }
        prevStatus = "l";

    }else if(touch.pageX < mTouchOffsetLeft + 2*mWidth/3){ // 前後移動

        if(touch.pageY < mTouchOffsetTop + mHeight/2){
            if(prevStatus != "f"){
                webiopi().callMacro("set6LegsAction", ["f", commandID++]);
            }
            prevStatus = "f";
        }else{
            if(prevStatus != "b"){
                webiopi().callMacro("set6LegsAction", ["b", commandID++]);
            }
            prevStatus = "b";
        }

    }else{ // 右旋回

        if(prevStatus != "r"){
            webiopi().callMacro("set6LegsAction", ["r", commandID++]);
        }
        prevStatus = "r";

    }
}

// タッチ終了時のイベントリスナー
function touchEndEvent(e){
    e.preventDefault();
    webiopi().callMacro("set6LegsAction", ["s", commandID++]);
    prevStatus = "s";
}

// クリック時のイベントリスナー（主にPC用）
function clickEvent(e){
    e.preventDefault();

    // タッチによるクリックは無視(IE)
    if(support.pointer || support.mspointer){
        if(e.pointerType == 'touch' || e.pointerType == 2){
            return;
        }
    }

    if(e.pageX < mTouchOffsetLeft ||
       e.pageX >= mTouchOffsetLeft + mWidth ||
       e.pageY < mTouchOffsetTop ||
       e.pageY >= mTouchOffsetTop + mHeight){

        return;
    }

    if(e.pageX < mTouchOffsetLeft + mWidth/3){ // 左旋回

        if(prevStatus != "l"){
            webiopi().callMacro("set6LegsAction", ["l", commandID++]);
        }
        prevStatus = "l";

    }else if(e.pageX < mTouchOffsetLeft + 2*mWidth/3){ // 前後移動

        if(e.pageY >= mTouchOffsetTop + 2*mHeight/5 &&
           e.pageY < mTouchOffsetTop + 3*mHeight/5){

            webiopi().callMacro("set6LegsAction", ["s", commandID++]);
            prevStatus = "s";

        }else if(e.pageY < mTouchOffsetTop + mHeight/2){
            if(prevStatus != "f"){
                webiopi().callMacro("set6LegsAction", ["f", commandID++]);
            }
            prevStatus = "f";

        }else{
            if(prevStatus != "b"){
                webiopi().callMacro("set6LegsAction", ["b", commandID++]);
            }
            prevStatus = "b";
        }

    }else{ // 右旋回

        if(prevStatus != "r"){
            webiopi().callMacro("set6LegsAction", ["r", commandID++]);
        }
        prevStatus = "r";

    }

}

function applyCustomCss(custom_css){
    var head = document.getElementsByTagName('head')[0];
    var style = document.createElement('link');
    style.rel = "stylesheet";
    style.type = 'text/css';
    style.href = custom_css;
    head.appendChild(style);
}

