import * as vscode from 'vscode';

export function activate(context: vscode.ExtensionContext) {
    console.log('FiaPhy Language Support activated');

    // Diagnostic collection for errors
    const diagnosticCollection = vscode.languages.createDiagnosticCollection('fiaphy');
    context.subscriptions.push(diagnosticCollection);

    // Register hover provider for documentation
    const hoverProvider = vscode.languages.registerHoverProvider(['cpp', 'c'], {
        provideHover(document, position, token) {
            const range = document.getWordRangeAtPosition(position);
            const word = document.getText(range);

            // FiaPhy function documentation
            const docs = getFiaPhyDocumentation(word);
            if (docs) {
                const markdown = new vscode.MarkdownString();
                markdown.isTrusted = true;
                markdown.supportHtml = true;
                markdown.appendMarkdown(`### ${docs.signature}\n\n`);
                markdown.appendMarkdown(`${docs.description}\n\n`);
                if (docs.example) {
                    markdown.appendCodeblock(docs.example, 'cpp');
                }
                markdown.appendMarkdown(`\n[📖 Full Documentation](${docs.url})`);
                return new vscode.Hover(markdown);
            }
        }
    });

    // Register completion provider
    const completionProvider = vscode.languages.registerCompletionItemProvider(['cpp', 'c'], {
        provideCompletionItems(document, position, token, context) {
            const linePrefix = document.lineAt(position).text.substr(0, position.character);
            
            // Check if we're in FiaPhy namespace
            if (!linePrefix.includes('FiaPhy::') && !linePrefix.includes('#include <FiaPhy.h>')) {
                return undefined;
            }

            return getFiaPhyCompletions();
        }
    }, ':');

    // Command: Open documentation
    const openDocsCommand = vscode.commands.registerCommand('fiaphy.openDocumentation', () => {
        vscode.env.openExternal(vscode.Uri.parse('https://github.com/fiaos-org/FiaPhy'));
    });

    // Command: Insert basic example
    const insertExampleCommand = vscode.commands.registerCommand('fiaphy.insertBasicExample', () => {
        const editor = vscode.window.activeTextEditor;
        if (editor) {
            const snippet = new vscode.SnippetString(getBasicExample());
            editor.insertSnippet(snippet);
        }
    });

    // Document change listener for error detection
    const documentChangeListener = vscode.workspace.onDidChangeTextDocument(event => {
        if (event.document.languageId === 'cpp' || event.document.languageId === 'c') {
            validateFiaPhyCode(event.document, diagnosticCollection);
        }
    });

    // Active editor change listener
    const editorChangeListener = vscode.window.onDidChangeActiveTextEditor(editor => {
        if (editor && (editor.document.languageId === 'cpp' || editor.document.languageId === 'c')) {
            validateFiaPhyCode(editor.document, diagnosticCollection);
        }
    });

    context.subscriptions.push(
        hoverProvider,
        completionProvider,
        openDocsCommand,
        insertExampleCommand,
        documentChangeListener,
        editorChangeListener
    );
}

function getFiaPhyDocumentation(symbol: string): any {
    const docs: { [key: string]: any } = {
        'DTDSS': {
            signature: 'class FiaPhy::DTDSS',
            description: 'Main DTDSS algorithm class for solar radiation reconstruction',
            url: 'https://github.com/fiaos-org/FiaPhy#quick-start',
            example: 'FiaPhy::DTDSS system;\nsystem.configure(lat, lon, alt);'
        },
        'configure': {
            signature: 'void configure(float latitude, float longitude, float altitude_m)',
            description: 'Configure geographic location for solar geometry calculations',
            url: 'https://github.com/fiaos-org/FiaPhy',
            example: 'system.configure(6.9271, 79.8612, 100.0);'
        },
        'feedReferenceData': {
            signature: 'void feedReferenceData(float temp_C, float humidity_RH, float pressure_hPa)',
            description: 'Feed sensor data from ventilated reference sensor',
            url: 'https://github.com/fiaos-org/FiaPhy',
            example: 'system.feedReferenceData(25.5, 60.0, 1013.25);'
        },
        'feedFluxData': {
            signature: 'void feedFluxData(float temp_C, float humidity_RH, float pressure_hPa)',
            description: 'Feed sensor data from black-body flux sensor',
            url: 'https://github.com/fiaos-org/FiaPhy',
            example: 'system.feedFluxData(28.3, 58.5, 1013.20);'
        },
        'isFrameReady': {
            signature: 'bool isFrameReady()',
            description: 'Check if complete sensor frame is ready for computation',
            url: 'https://github.com/fiaos-org/FiaPhy',
            example: 'if (system.isFrameReady()) { /* compute */ }'
        },
        'compute': {
            signature: 'RadiationResult compute()',
            description: 'Compute solar radiation and heat flux from buffered sensor data',
            url: 'https://github.com/fiaos-org/FiaPhy',
            example: 'FiaPhy::RadiationResult result = system.compute();'
        },
        'saturationVaporPressure': {
            signature: 'float saturationVaporPressure(float temp_C)',
            description: 'Calculate saturation vapor pressure using Magnus formula (Section 2.3)',
            url: 'https://github.com/fiaos-org/FiaPhy',
            example: 'float e_s = FiaPhy::Thermodynamics::saturationVaporPressure(25.0f);'
        },
        'airDensity': {
            signature: 'float airDensity(float pressure_hPa, float temp_C, float humidity_RH)',
            description: 'Calculate moist air density (altitude-adaptive, Section 2.2)',
            url: 'https://github.com/fiaos-org/FiaPhy',
            example: 'float rho = FiaPhy::Thermodynamics::airDensity(1013.25, 25.0, 50.0);'
        }
    };

    return docs[symbol];
}

function getFiaPhyCompletions(): vscode.CompletionItem[] {
    const completions: vscode.CompletionItem[] = [];

    // Main class
    const dtdssClass = new vscode.CompletionItem('DTDSS', vscode.CompletionItemKind.Class);
    dtdssClass.detail = 'FiaPhy::DTDSS';
    dtdssClass.documentation = 'Main DTDSS algorithm class';
    completions.push(dtdssClass);

    // Methods
    const methods = [
        { name: 'configure', detail: 'Configure location', snippet: 'configure(${1:latitude}, ${2:longitude}, ${3:altitude})' },
        { name: 'feedReferenceData', detail: 'Feed reference sensor', snippet: 'feedReferenceData(${1:temp}, ${2:humidity}, ${3:pressure})' },
        { name: 'feedFluxData', detail: 'Feed flux sensor', snippet: 'feedFluxData(${1:temp}, ${2:humidity}, ${3:pressure})' },
        { name: 'isFrameReady', detail: 'Check frame ready', snippet: 'isFrameReady()' },
        { name: 'compute', detail: 'Compute radiation', snippet: 'compute()' }
    ];

    methods.forEach(m => {
        const item = new vscode.CompletionItem(m.name, vscode.CompletionItemKind.Method);
        item.detail = m.detail;
        item.insertText = new vscode.SnippetString(m.snippet);
        completions.push(item);
    });

    return completions;
}

function validateFiaPhyCode(document: vscode.TextDocument, diagnosticCollection: vscode.DiagnosticCollection) {
    const config = vscode.workspace.getConfiguration('fiaphy');
    if (!config.get('enableDiagnostics')) {
        return;
    }

    const diagnostics: vscode.Diagnostic[] = [];
    const text = document.getText();

    // Check for deprecated functions
    const deprecatedFunctions = [
        'calculateSaturationPressure',
        'calculateVaporPressure',
        'calculateAirDensity',
        'calculateMixingRatio',
        'calculateSpecificEnthalpy',
        'estimateConvectiveCoefficient'
    ];

    for (let i = 0; i < document.lineCount; i++) {
        const line = document.lineAt(i);
        const lineText = line.text;

        deprecatedFunctions.forEach(func => {
            if (lineText.includes(func)) {
                const index = lineText.indexOf(func);
                const range = new vscode.Range(i, index, i, index + func.length);
                const diagnostic = new vscode.Diagnostic(
                    range,
                    `'${func}' is deprecated. Use renamed function (v1.0.1+)`,
                    vscode.DiagnosticSeverity.Warning
                );
                diagnostic.source = 'FiaPhy';
                diagnostics.push(diagnostic);
            }
        });

        // Check for missing configure() call
        if (lineText.includes('system.compute()') && !text.includes('system.configure(')) {
            const index = lineText.indexOf('compute');
            const range = new vscode.Range(i, index, i, index + 7);
            const diagnostic = new vscode.Diagnostic(
                range,
                'Call configure() before compute()',
                vscode.DiagnosticSeverity.Error
            );
            diagnostic.source = 'FiaPhy';
            diagnostics.push(diagnostic);
        }

        // Check for frame ready pattern
        if (lineText.includes('.compute()') && !lineText.includes('isFrameReady')) {
            const prevLines = text.substring(0, document.offsetAt(line.range.start));
            if (!prevLines.includes('isFrameReady()')) {
                const index = lineText.indexOf('compute');
                const range = new vscode.Range(i, index, i, index + 7);
                const diagnostic = new vscode.Diagnostic(
                    range,
                    'Check isFrameReady() before calling compute()',
                    vscode.DiagnosticSeverity.Warning
                );
                diagnostic.source = 'FiaPhy';
                diagnostics.push(diagnostic);
            }
        }
    }

    diagnosticCollection.set(document.uri, diagnostics);
}

function getBasicExample(): string {
    return `#include <FiaPhy.h>
#include <Adafruit_BME280.h>

Adafruit_BME280 refSensor;   // Address 0x76
Adafruit_BME280 fluxSensor;  // Address 0x77
FiaPhy::DTDSS system;

void setup() {
    Serial.begin(115200);
    
    refSensor.begin(0x76);
    fluxSensor.begin(0x77);
    
    system.configure(\${1:6.9271}, \${2:79.8612}, \${3:100.0});
}

void loop() {
    system.feedReferenceData(refSensor.readTemperature(), 
                            refSensor.readHumidity(), 
                            refSensor.readPressure() / 100.0);
    
    system.feedFluxData(fluxSensor.readTemperature(), 
                       fluxSensor.readHumidity(), 
                       fluxSensor.readPressure() / 100.0);
    
    if (system.isFrameReady()) {
        FiaPhy::RadiationResult result = system.compute();
        if (result.valid) {
            Serial.print("GHI: ");
            Serial.print(result.ghi_Wm2);
            Serial.println(" W/m²");
        }
    }
    
    delay(3000);
}`;
}

export function deactivate() {}
