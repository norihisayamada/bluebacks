function initialize_webiopi(){
    // webiopiの準備が終わってからstyles.cssを適用する
    applyCustomCss('styles.css');

    // GPIOの状態を監視しない
    webiopi().refreshGPIO(false);
}

function sendCommand(group, command){
    // script.py内のsendCommand関数を実行。
    webiopi().callMacro("sendCommand", [group, command]);
}

function applyCustomCss(custom_css){
    var head = document.getElementsByTagName('head')[0];
    var style = document.createElement('link');
    style.rel = "stylesheet";
    style.type = 'text/css';
    style.href = custom_css;
    head.appendChild(style);
}

