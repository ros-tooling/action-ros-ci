module.exports = {
	clearMocks: true,
	// Removing 'js' throws a Validation Error: moduleFileExtensions must include 'js'
	moduleFileExtensions: ["ts", "js"],
	resolver: "<rootDir>/jest.resolver.js",
	testEnvironment: "node",
	testMatch: ["**/__tests__/*.test.ts", "**/*.test.ts"],
	testRunner: "jest-circus/runner",
	transform: {
		"^.+\\.ts$": "ts-jest",
		// Transpile the ESM-only dependencies to CommonJS
		"^.+\\.js$": [
			"ts-jest",
			{ tsconfig: { allowJs: true, isolatedModules: true } },
		],
	},
	// Packages not listed here are left as is, so ESM-only dependencies must be listed
	transformIgnorePatterns: [
		"/node_modules/(?!(@actions|@octokit|before-after-hook|content-type|universal-user-agent)/)",
	],
	verbose: true,
};
