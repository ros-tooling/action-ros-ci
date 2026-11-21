module.exports = {
	clearMocks: true,
	// Removing 'js' throws a Validation Error: moduleFileExtensions must include 'js'
	moduleFileExtensions: ["ts", "js"],
	testEnvironment: "node",
	testMatch: ["**/__tests__/*.test.ts", "**/*.test.ts"],
	testRunner: "jest-circus/runner",
	transform: {
		"^.+\\.ts$": "ts-jest",
	},
	transformIgnorePatterns: ["^.+\\.js$"],
	verbose: true,
};
