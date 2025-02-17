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

var mTouchWidth;
var mTouchHeight;
var mTouchOffsetTop;
var mTouchOffsetLeft;

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
       touch.pageX >= mTouchOffsetLeft + mTouchWidth ||
       touch.pageY < mTouchOffsetTop ||
       touch.pageY >= mTouchOffsetTop + mTouchHeight){

        return;
    }

    if(touch.pageX < mTouchOffsetLeft + mTouchWidth/3){ // 左旋回

        if(prevStatus != "l"){
            webiopi().callMacro("set6LegsAction", ["l", commandID++]);
        }
        prevStatus = "l";

    }else if(touch.pageX < mTouchOffsetLeft + 2*mTouchWidth/3){ // 前後移動

        if(touch.pageY < mTouchOffsetTop + mTouchHeight/2){
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
       e.pageX >= mTouchOffsetLeft + mTouchWidth ||
       e.pageY < mTouchOffsetTop ||
       e.pageY >= mTouchOffsetTop + mTouchHeight){

        return;
    }

    if(e.pageX < mTouchOffsetLeft + mTouchWidth/3){ // 左旋回

        if(prevStatus != "l"){
            webiopi().callMacro("set6LegsAction", ["l", commandID++]);
        }
        prevStatus = "l";

    }else if(e.pageX < mTouchOffsetLeft + 2*mTouchWidth/3){ // 前後移動

        if(e.pageY >= mTouchOffsetTop + 2*mTouchHeight/5 &&
           e.pageY < mTouchOffsetTop + 3*mTouchHeight/5){

            webiopi().callMacro("set6LegsAction", ["s", commandID++]);
            prevStatus = "s";

        }else if(e.pageY < mTouchOffsetTop + mTouchHeight/2){
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

function resize_canvas(){

    if($(window).width() < $(window).height()){
        isPortrait = true;
    }else{
        isPortrait = false;
    }

    if(isPortrait){
        mTouchWidth = 0.95*$(window).width();
        mTouchHeight = mTouchWidth;
    }else{
        mTouchHeight = 0.95*$(window).height();
        mTouchWidth = mTouchHeight;
    }

    mTouchOffsetLeft = ($(window).width()-mTouchWidth)/2;
    mTouchOffsetTop = $("#touchArea").offset().top;

    $( "#touchImage" ).width(mTouchWidth);
    $( "#touchImage" ).height(mTouchHeight);
}

function applyCustomCss(custom_css){
    var head = document.getElementsByTagName('head')[0];
    var style = document.createElement('link');
    style.rel = "stylesheet";
    style.type = 'text/css';
    style.href = custom_css;
    head.appendChild(style);
}

